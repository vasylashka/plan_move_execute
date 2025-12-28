#!/usr/bin/env python3
"""
xArm7 Physics Simulator (Final Production Version)
- Features: Real-time Physics, Collision Stop, Error Locking, Custom Home.
- Fixes: Inverted Gripper, Explicit Error Messages in Service Responses.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import pybullet as p
import pybullet_data
import time
from threading import Thread, Lock, Event

# ROS 2 Messages & Services
from sensor_msgs.msg import JointState
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import (
    Call, SetInt16, MoveJoint, MoveCartesian, MoveHome,
    GripperMove
)


class XArm7PhysicsSim(Node):
    def __init__(self):
        super().__init__('xarm7_physics_sim')

        # ===== USER CONFIG =====
        # Custom Home Position (Safe "Cobra" Pose)
        self.home_angles = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0]

        # ===== PARAMETERS =====
        self.declare_parameter('urdf_path', '/home/ros2_ws/plan_move_execute/src/assets/models/xarm7.urdf')
        self.declare_parameter('hw_ns', 'xarm')
        self.declare_parameter('frequency', 240.0)

        self.urdf_path = self.get_parameter('urdf_path').value
        self.hw_ns = self.get_parameter('hw_ns').value
        self.sim_freq = self.get_parameter('frequency').value

        # ===== STATE VARIABLES =====
        self.physics_client = None
        self.robot_id = None
        self.sim_lock = Lock()
        self.running = True
        self.collision_event = Event()
        self.is_moving = False

        self.arm_indices = []
        self.drive_joint_idx = None
        self.mimic_indices = []
        self.tcp_link_idx = -1
        self.base_link_index = -1
        self.joint_limits = {}

        self.dof = 7
        self.curr_joints = list(self.home_angles)
        self.curr_pose = [0.0] * 6

        # Robot Status: 1=Idle, 2=Moving, 4=Error
        self.robot_state = 1
        self.error_code = 0

        self._init_physics()
        self._init_ros()

        self.get_logger().info(f"xArm Simulator Online. Home Pose: {self.home_angles}")

    def _init_physics(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")

        try:
            self.robot_id = p.loadURDF(
                self.urdf_path, [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
            )
        except Exception as e:
            self.get_logger().error(f"FATAL: Failed to load URDF: {e}")
            return

        # --- Parse Joints ---
        num_joints = p.getNumJoints(self.robot_id)
        mimic_names = ["left_finger_joint", "left_inner_knuckle_joint",
                       "right_outer_knuckle_joint", "right_finger_joint",
                       "right_inner_knuckle_joint"]

        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode('utf-8')
            link_name = info[12].decode('utf-8')

            if "link_base" in link_name: self.base_link_index = i
            if "joint" in name and name[-1].isdigit():
                self.arm_indices.append(i)
                self.joint_limits[i] = (info[8], info[9])
                p.enableJointForceTorqueSensor(self.robot_id, i, enableSensor=1)
            if name == "drive_joint": self.drive_joint_idx = i
            if name in mimic_names: self.mimic_indices.append(i)
            if link_name == "link_tcp": self.tcp_link_idx = i

        self.arm_indices.sort(key=lambda x: p.getJointInfo(self.robot_id, x)[1])
        self._disable_gripper_collisions()

        # --- Set Initial Pose (Home) ---
        for i, angle in enumerate(self.home_angles):
            if i < len(self.arm_indices):
                p.resetJointState(self.robot_id, self.arm_indices[i], angle)

        p.setJointMotorControlArray(
            self.robot_id, self.arm_indices, p.POSITION_CONTROL,
            targetPositions=self.home_angles,
            forces=[200.0] * 7, positionGains=[0.05] * 7
        )

        # Default Gripper Open
        if self.drive_joint_idx is not None:
            all_g = [self.drive_joint_idx] + self.mimic_indices
            p.setJointMotorControlArray(self.robot_id, all_g, p.POSITION_CONTROL,
                                        targetPositions=[0.0] * len(all_g), forces=[50] * len(all_g))

        self.sim_thread = Thread(target=self._physics_loop, daemon=True)
        self.sim_thread.start()

    def _disable_gripper_collisions(self):
        if self.drive_joint_idx is None: return
        links = [self.drive_joint_idx] + self.mimic_indices
        for i in links:
            for j in links:
                if i != j: p.setCollisionFilterPair(self.robot_id, self.robot_id, i, j, enableCollision=0)

    # ================= PHYSICS LOOP =================
    def _physics_loop(self):
        dt = 1.0 / self.sim_freq
        while self.running and rclpy.ok():
            with self.sim_lock:
                p.stepSimulation()

                # Update Cache
                states = p.getJointStates(self.robot_id, self.arm_indices)
                self.curr_joints = [s[0] for s in states]

                if self.tcp_link_idx != -1:
                    ls = p.getLinkState(self.robot_id, self.tcp_link_idx)
                    self.curr_pose = list(ls[0]) + list(p.getEulerFromQuaternion(ls[1]))

                # Real-Time Collision Check
                if self.is_moving:
                    contacts = p.getContactPoints(self.robot_id)
                    for c in contacts:
                        link_a = c[3]
                        body_b = c[2]
                        # Ignore base hitting floor
                        if body_b == self.plane_id:
                            if link_a != self.base_link_index and link_a != -1:
                                self.get_logger().error(f"CRASH: Link {link_a} hit the floor!")
                                self.collision_event.set()
                        # Hit other object
                        elif body_b != self.robot_id:
                            self.get_logger().error(f"CRASH: Link {link_a} hit object {body_b}!")
                            self.collision_event.set()
            time.sleep(dt)

    # ================= WAIT HELPER =================
    def _wait_for_arrival(self, target_joints, timeout=10.0):
        start_time = time.time()
        self.is_moving = True
        self.robot_state = 2  # Moving

        try:
            while time.time() - start_time < timeout:
                # 1. Collision Detected?
                if self.collision_event.is_set():
                    self.is_moving = False
                    self.robot_state = 4  # Error State
                    self.error_code = 1  # Standard Error

                    # FREEZE ROBOT
                    with self.sim_lock:
                        p.setJointMotorControlArray(
                            self.robot_id, self.arm_indices, p.POSITION_CONTROL,
                            targetPositions=self.curr_joints, forces=[200.0] * 7, positionGains=[0.1] * 7
                        )
                    return 1  # Return Collision Code

                # 2. Arrived?
                error = sum([abs(self.curr_joints[i] - target_joints[i]) for i in range(7)])
                if error < 0.05:
                    self.is_moving = False
                    self.robot_state = 1  # Idle
                    return 0  # Success

                time.sleep(0.05)

            self.is_moving = False
            return -1  # Timeout
        except:
            self.is_moving = False
            return -1

    # ================= SERVICES =================

    def _srv_move_joint(self, req, res):
        # 1. Safety Check
        if self.error_code != 0:
            res.ret = 1
            res.message = "ERROR: Robot is Locked. Please call clean_error first."
            return res

        target = list(req.angles[:7])
        self.collision_event.clear()

        with self.sim_lock:
            p.setJointMotorControlArray(
                self.robot_id, self.arm_indices, p.POSITION_CONTROL,
                targetPositions=target, forces=[200.0] * 7, positionGains=[0.05] * 7
            )

        status = self._wait_for_arrival(target)

        if status == 0:
            res.ret = 0
            res.message = "Move Success"
        elif status == 1:
            res.ret = 1
            res.message = "CRASH DETECTED: Movement Aborted"
        else:
            res.ret = -1
            res.message = "Timeout: Target not reached"
        return res

    def _srv_move_cartesian(self, req, res):
        # 1. Safety Check
        if self.error_code != 0:
            res.ret = 1
            res.message = "ERROR: Robot is Locked. Please call clean_error first."
            return res

        pos, rpy = req.pose[:3], req.pose[3:6]
        quat = p.getQuaternionFromEuler(rpy)
        self.collision_event.clear()

        with self.sim_lock:
            target = p.calculateInverseKinematics(
                self.robot_id, self.tcp_link_idx, pos, quat,
                maxNumIterations=500, residualThreshold=1e-5,
                lowerLimits=[self.joint_limits[i][0] for i in self.arm_indices],
                upperLimits=[self.joint_limits[i][1] for i in self.arm_indices],
                jointRanges=[6.28] * 7, restPoses=self.home_angles
            )[:7]
            p.setJointMotorControlArray(
                self.robot_id, self.arm_indices, p.POSITION_CONTROL,
                targetPositions=target, forces=[200.0] * 7, positionGains=[0.05] * 7
            )

        status = self._wait_for_arrival(target)

        if status == 0:
            res.ret = 0
            res.message = "Move Success"
        elif status == 1:
            res.ret = 1
            res.message = "CRASH DETECTED: Movement Aborted"
        else:
            res.ret = -1
            res.message = "Timeout or Unreachable"
        return res

    def _srv_move_home(self, req, res):
        if self.error_code != 0:
            res.ret = 1
            res.message = "ERROR: Robot is Locked"
            return res

        self.collision_event.clear()
        target = list(self.home_angles)

        with self.sim_lock:
            p.setJointMotorControlArray(self.robot_id, self.arm_indices, p.POSITION_CONTROL,
                                        targetPositions=target, forces=[200.0] * 7, positionGains=[0.05] * 7)

        status = self._wait_for_arrival(target)
        if status == 0:
            res.ret = 0
            res.message = "Arrived Home"
        else:
            res.ret = 1
            res.message = "Failed to go Home"
        return res

    def _srv_gripper(self, req, res):
        if self.error_code != 0:
            res.ret = 1
            res.message = "ERROR: Robot Locked"
            return res

        normalized = max(0.0, min(0.85, req.pos / 1000.0))
        target_rad = 0.85 - normalized  # Invert Logic

        with self.sim_lock:
            if self.drive_joint_idx is not None:
                all_g = [self.drive_joint_idx] + self.mimic_indices
                p.setJointMotorControlArray(
                    self.robot_id, all_g, p.POSITION_CONTROL,
                    targetPositions=[target_rad] * len(all_g),
                    forces=[50.0] * len(all_g), positionGains=[0.5] * len(all_g)
                )
        res.ret = 0
        res.message = "Gripper Moved"
        return res

    def _srv_clean_error(self, req, res):
        self.error_code = 0
        self.robot_state = 1
        self.collision_event.clear()
        res.ret = 0
        res.message = "Errors Cleared. Robot Unlocked."
        self.get_logger().info("System Reset: Errors Cleared.")
        return res

    # Dummy services
    def _srv_ok(self, req, res):
        res.ret = 0
        res.message = "OK"
        return res

    # ================= INIT ROS =================
    def _init_ros(self):
        cb = MutuallyExclusiveCallbackGroup()
        self.pub_joints = self.create_publisher(JointState, f'{self.hw_ns}/joint_states', 10)
        self.pub_robot = self.create_publisher(RobotMsg, f'{self.hw_ns}/robot_states', 10)

        self.create_service(MoveJoint, f'{self.hw_ns}/set_servo_angle', self._srv_move_joint, callback_group=cb)
        self.create_service(MoveCartesian, f'{self.hw_ns}/set_position', self._srv_move_cartesian, callback_group=cb)
        self.create_service(MoveHome, f'{self.hw_ns}/move_gohome', self._srv_move_home, callback_group=cb)
        self.create_service(GripperMove, f'{self.hw_ns}/set_gripper_position', self._srv_gripper, callback_group=cb)
        self.create_service(Call, f'{self.hw_ns}/clean_error', self._srv_clean_error, callback_group=cb)
        for s in ['set_mode', 'set_state', 'clean_warn']:
            t = SetInt16 if 'set' in s else Call
            self.create_service(t, f'{self.hw_ns}/{s}', self._srv_ok, callback_group=cb)
        self.create_timer(0.03, self._publish_status)

    def _publish_status(self):
        now = self.get_clock().now().to_msg()
        js = JointState();
        js.header.stamp = now;
        js.name = [f'joint{i + 1}' for i in range(7)]
        js.position = self.curr_joints
        self.pub_joints.publish(js)
        rs = RobotMsg();
        rs.header.stamp = now;
        rs.state = self.robot_state;
        rs.err = self.error_code
        rs.angle = self.curr_joints;
        rs.pose = self.curr_pose
        self.pub_robot.publish(rs)

    def destroy_node(self):
        self.running = False
        if self.physics_client: p.disconnect(self.physics_client)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XArm7PhysicsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()