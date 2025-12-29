import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import json
import numpy as np
from threading import Lock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from xarm_local_msgs.srv import GetApfDistances, LoadScenario, CheckCollision
from xarm_local_msgs.msg import ApfLinkData


class XArmPlanningScene(Node):
    def __init__(self):
        super().__init__('xarm_planning_scene')

        self.declare_parameter('urdf_path', '/home/ros2_ws/plan_move_execute/src/assets/models/xarm7.urdf')
        self.urdf_path = self.get_parameter('urdf_path').value

        # Load Scenarios
        try:
            with open("/home/ros2_ws/plan_move_execute/src/xarm_local_planner/xarm_local_planner/scenarios.json",
                      "r") as f:
                self.scenarios = json.load(f)
        except Exception as e:
            self.scenarios = []
            self.get_logger().warn(f"scenarios.json not found: {e}")

        # PyBullet Setup
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(self.urdf_path, [0, 0, 0], useFixedBase=True)

        self.sim_lock = Lock()
        self.obstacle_ids = []
        self.target_id = None
        self.current_joints = [0.0] * 7

        # ===== OPTIMIZED JOINT INDICES =====
        self.arm_indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            name = p.getJointInfo(self.robot_id, i)[1].decode('utf-8')
            if "joint" in name and any(char.isdigit() for char in name):
                self.arm_indices.append(i)
        self.arm_indices = self.arm_indices[:7]  # Ensure exactly 7

        self.movable_indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[3] > -1:
                self.movable_indices.append(i)

        self.tcp_link_idx = 7
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[12].decode('utf-8') == 'link_tcp':
                self.tcp_link_idx = i

        # Publisher for Target Joints
        self.target_pub = self.create_publisher(Float32MultiArray, '/planning/target_joints', 10)
        self.current_target_joints = [0.0] * 7

        # ROS Interface
        self.create_subscription(JointState, '/xarm/joint_states', self._sync_callback, 10)
        self.create_service(LoadScenario, '/planning/load_scenario', self._srv_load_scenario)
        self.create_service(CheckCollision, '/planning/check_collision', self._srv_check_collision)
        self.create_service(GetApfDistances, '/planning/get_apf_distances', self._srv_get_apf_data)

        self.load_scenario(0)
        self.get_logger().info("Planning Engine Ready.")

    def load_scenario(self, scenario_id):
        if scenario_id >= len(self.scenarios): return
        data = self.scenarios[scenario_id]

        with self.sim_lock:
            for uid in self.obstacle_ids: p.removeBody(uid)
            self.obstacle_ids = []
            if self.target_id is not None:
                p.removeBody(self.target_id)
                self.target_id = None

            for obs in data['obstacles']:
                pos = obs['pos']
                if obs['type'] == 'box':
                    h = [d / 2 for d in obs['dim']]
                    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=h)
                    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=h, rgbaColor=[0.5, 0.5, 0.5, 1])
                else:
                    col = p.createCollisionShape(p.GEOM_SPHERE, radius=obs['radius'])
                    vis = p.createVisualShape(p.GEOM_SPHERE, radius=obs['radius'], rgbaColor=[0.5, 0.5, 0.5, 1])
                uid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                        basePosition=pos)
                self.obstacle_ids.append(uid)

            if 'target_pose' in data:
                tgt_raw = data['target_pose']
                if len(tgt_raw) < 6: tgt_raw += [0.0] * (6 - len(tgt_raw))
                t_pos = tgt_raw[:3]
                t_quat = p.getQuaternionFromEuler(tgt_raw[3:6])

                t_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 1, 0, 0.8])
                self.target_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=t_vis, basePosition=t_pos)

                joints = p.calculateInverseKinematics(
                    self.robot_id, self.tcp_link_idx, t_pos, t_quat,
                    lowerLimits=[-3.14] * 7, upperLimits=[3.14] * 7,
                    jointRanges=[6.28] * 7, restPoses=list(self.current_joints)
                )
                self.current_target_joints = list(joints[:7])

            self.get_logger().info(f"Loaded Scenario {scenario_id}")

    def _sync_callback(self, msg):
        if len(msg.position) >= 7:
            with self.sim_lock:
                self.current_joints = msg.position[:7]
                for i, angle in enumerate(self.current_joints):
                    p.resetJointState(self.robot_id, self.arm_indices[i], angle)

        out_msg = Float32MultiArray()
        out_msg.data = [float(x) for x in self.current_target_joints]
        self.target_pub.publish(out_msg)

    def _srv_load_scenario(self, req, res):
        if 0 <= req.scenario_index < len(self.scenarios):
            self.load_scenario(req.scenario_index)
            res.success = True
            res.message = f"Loaded {req.scenario_index}"
        else:
            res.success = False
            res.message = "Invalid ID"
        return res

    def _srv_check_collision(self, req, res):
        if len(req.joint_positions) < 7:
            res.collision_detected = False
            return res

        is_hit = False
        with self.sim_lock:
            for i in range(7): p.resetJointState(self.robot_id, self.arm_indices[i], req.joint_positions[i])
            p.performCollisionDetection()
            for c in p.getContactPoints(self.robot_id):
                if c[2] == self.plane_id and c[3] <= 1: continue
                is_hit = True
                break
            for i, val in enumerate(self.current_joints): p.resetJointState(self.robot_id, self.arm_indices[i], val)

        res.collision_detected = is_hit
        return res

    def _srv_get_apf_data(self, req, res):
        # Critical links for collision checking (Arm)
        critical_links = [2, 4, 6, 9, 11, 14]
        link_names = {
            2: "Shoulder", 4: "Elbow", 6: "Wrist",
            9: "Hand Base", 11: "Left Finger", 14: "Right Finger"
        }
        targets = [self.plane_id] + self.obstacle_ids

        states = p.getJointStates(self.robot_id, self.movable_indices)
        m_pos = [s[0] for s in states]
        zero_vec = [0.0] * len(m_pos)

        with self.sim_lock:
            for link_idx in critical_links:
                min_dist = 100.0
                best_vec = [0.0, 0.0, 0.0]
                p_robot_world = [0, 0, 0]
                found = False

                for target_id in targets:
                    if target_id == self.robot_id: continue
                    results = p.getClosestPoints(bodyA=self.robot_id, bodyB=target_id, distance=10.0,
                                                 linkIndexA=link_idx)
                    if not results: continue
                    for c in results:
                        dist = c[8]
                        if dist < min_dist:
                            min_dist = dist
                            found = True
                            p_robot_world = c[5]
                            p_obs = c[6]
                            best_vec = (np.array(p_obs) - np.array(p_robot_world)).tolist()

                if found:
                    msg = ApfLinkData()
                    msg.link_index = int(link_idx)
                    msg.link_name = link_names.get(link_idx, f"Link_{link_idx}")
                    msg.min_distance = float(min_dist)
                    msg.direction_vector.x = float(best_vec[0])
                    msg.direction_vector.y = float(best_vec[1])
                    msg.direction_vector.z = float(best_vec[2])

                    ls = p.getLinkState(self.robot_id, link_idx)
                    inv_p, inv_r = p.invertTransform(ls[0], ls[1])
                    local_p, _ = p.multiplyTransforms(inv_p, inv_r, p_robot_world, [0, 0, 0, 1])

                    jac_t, _ = p.calculateJacobian(
                        self.robot_id, link_idx, local_p,
                        m_pos, zero_vec, zero_vec
                    )

                    flat_jac = []
                    for row in jac_t: flat_jac.extend(row)
                    msg.jacobian = flat_jac

                    res.data.append(msg)
        return res


def main():
    rclpy.init()
    rclpy.spin(XArmPlanningScene())
    rclpy.shutdown()


if __name__ == '__main__':
    main()