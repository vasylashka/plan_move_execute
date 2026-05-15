import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Thread

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from xarm_local_msgs.srv import GetApfDistances
from xarm_msgs.srv import MoveJoint, SetInt16
from xarm_msgs.msg import RobotMsg  # ADDED


class BaseLocalPlanner(Node):
    def __init__(self, node_name='local_planner'):
        super().__init__(node_name)

        # Shared Parameters
        self.declare_parameter('step_size', 0.03)
        self.alpha = self.get_parameter('step_size').value

        self.current_joints = np.zeros(7)
        self.q_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Cartesian State Variables # ADDED
        self.current_ee_pos = np.zeros(3)
        self.target_ee_pos = np.zeros(3)

        self.planning_active = False
        self.goal_reached_flag = False

        # ROS Communications
        self.sub_joints = self.create_subscription(JointState, '/xarm/joint_states', self.joint_callback, 10)
        self.sub_goal = self.create_subscription(Float32MultiArray, '/planning/target_joints', self.goal_callback, 10)

        # New Cartesian Subscriptions # ADDED
        self.sub_robot = self.create_subscription(RobotMsg, '/xarm/robot_states', self.robot_callback, 10)
        self.sub_target_pose = self.create_subscription(Float32MultiArray, '/planning/target_pose',
                                                        self.target_pose_callback, 10)

        # Telemetry Publishers
        self.pub_telemetry = self.create_publisher(Float32MultiArray, '/planning/telemetry', 10)
        self.pub_status = self.create_publisher(Bool, '/planning/status_reached', 10)

        self.cli_env = self.create_client(GetApfDistances, '/planning/get_apf_distances')
        self.cli_move = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.cli_state = self.create_client(SetInt16, '/xarm/set_state')

        self.strategy = None
        self.planning_active = True
        self.plan_thread = Thread(target=self.control_loop)
        self.plan_thread.start()

    def joint_callback(self, msg):
        if len(msg.position) >= 7:
            self.current_joints = np.array(msg.position[:7])

    def robot_callback(self, msg):  # ADDED
        """Updates current Cartesian position from simulator ground truth."""
        if len(msg.pose) >= 3:
            self.current_ee_pos = np.array(msg.pose[:3])

    def target_pose_callback(self, msg):  # ADDED
        """Updates the Cartesian target from the planning scene."""
        if len(msg.data) >= 3:
            self.target_ee_pos = np.array(msg.data[:3])

    def goal_callback(self, msg):
        if len(msg.data) >= 7:
            new_goal = np.array(msg.data[:7])
            if np.linalg.norm(new_goal - self.q_goal) > 1e-3:
                self.q_goal = new_goal
                self.goal_reached_flag = False
                self.get_logger().info(f"New Goal Received: {np.round(self.q_goal, 3)}")
                self.flush_motion_queue()

    def flush_motion_queue(self):
        if self.cli_state.wait_for_service(timeout_sec=0.1):
            req_stop = SetInt16.Request()
            req_stop.data = 4
            self.cli_state.call_async(req_stop)

            req_ready = SetInt16.Request()
            req_ready.data = 0
            self.cli_state.call_async(req_ready)

        self.destroy_client(self.cli_move)
        self.cli_move = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.get_logger().info("Motion queues flushed.")

    def get_environment_data(self):
        req = GetApfDistances.Request()
        future = self.cli_env.call_async(req)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 1.0: return None
            time.sleep(0.01)
        return future.result()

    def control_loop(self):
        """The main execution loop with Cartesian distance stopping criteria"""
        time.sleep(2.0)
        while rclpy.ok() and self.planning_active:
            if self.strategy is None:
                continue

            cart_dist = np.linalg.norm(self.current_ee_pos - self.target_ee_pos)

            if cart_dist < 0.025:
                if not self.goal_reached_flag:
                    self.get_logger().info(f"Target Reached! Cartesian Distance: {cart_dist:.4f}m")
                    self.goal_reached_flag = True
                    self.pub_status.publish(Bool(data=True))
                time.sleep(0.1)
                continue
            else:
                if self.goal_reached_flag:
                    self.goal_reached_flag = False
                    self.pub_status.publish(Bool(data=False))

            t0 = time.time()
            env_data = self.get_environment_data()

            velocity_vector = self.strategy.compute_velocity(
                self.current_joints,
                self.q_goal,
                env_data
            )

            compute_duration = time.time() - t0

            min_clearance = 999.0
            if env_data and hasattr(env_data, 'data'):
                for link_data in env_data.data:
                    if link_data.min_distance < min_clearance:
                        min_clearance = link_data.min_distance

            telemetry_msg = Float32MultiArray()
            telemetry_msg.data = [float(compute_duration), float(min_clearance), float(cart_dist)]
            self.pub_telemetry.publish(telemetry_msg)

            if velocity_vector is not None:
                velocity_vector = np.clip(velocity_vector, -1.0, 1.0)
                q_next = self.current_joints + (velocity_vector * self.alpha)
                self.send_command(q_next)

            time.sleep(0.05)

    def send_command(self, q_target):
        req = MoveJoint.Request()
        req.angles = q_target.tolist()
        req.speed = 0.0
        req.wait = False
        self.cli_move.call_async(req)