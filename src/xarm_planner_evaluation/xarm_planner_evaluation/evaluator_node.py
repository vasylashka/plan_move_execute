import rclpy
from rclpy.node import Node
import time
import json
import random
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from xarm_msgs.msg import RobotMsg
from xarm_local_msgs.srv import LoadScenario
from xarm_msgs.srv import Call
from xarm_planner_evaluation.metrics import TrajectoryMetrics


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator_node')
        self.get_logger().info("Evaluator Node Initialized. Waiting for services...")

        # Declare parameters
        self.declare_parameter('scenarios_to_run', 5)
        self.declare_parameter('random_selection', False)
        self.declare_parameter('total_scenarios', 100)
        self.declare_parameter('scenario_config_path',
                               '/home/ros2_ws/plan_move_execute/src/xarm_local_planner/xarm_local_planner/scenarios.json')

        self.limit = self.get_parameter('scenarios_to_run').value
        self.is_random = self.get_parameter('random_selection').value
        self.total_scenarios = self.get_parameter('total_scenarios').value

        # Load Scenario JSON for metrics
        self.scenarios_data = self._load_scenarios()

        # Generate scenario list
        if self.is_random:
            sample_size = min(self.limit, self.total_scenarios)
            self.scenarios_to_execute = random.sample(range(self.total_scenarios), sample_size)
            self.limit = sample_size
            self.get_logger().info(f"Random mode ON. Selected: {self.scenarios_to_execute}")
        else:
            self.scenarios_to_execute = list(range(self.limit))
            self.get_logger().info(f"Sequential mode ON. Running first {self.limit}.")

        self.current_run_index = -1
        self.current_scenario_id = -1
        self.results = []

        # State Management Flags
        self.is_recording = False
        self.scenario_active = False
        self.is_resetting = False

        self.latest_latency = 0.0
        self.latest_clearance = 10.0
        self.latest_ee_pose = np.zeros(6)
        self.latest_target_joints = np.zeros(7)
        self.current_metrics = None

        # ROS Interfaces
        self.sub_joints = self.create_subscription(
            JointState, '/xarm/joint_states', self._cb_joints, 10)
        self.sub_robot = self.create_subscription(
            RobotMsg, '/xarm/robot_states', self._cb_robot_status, 10)
        self.sub_telemetry = self.create_subscription(
            Float32MultiArray, '/planning/telemetry', self._cb_telemetry, 10)
        self.sub_status = self.create_subscription(
            Bool, '/planning/status_reached', self._cb_status, 10)
        self.sub_target_joints = self.create_subscription(
            Float32MultiArray, '/planning/target_joints', self._cb_target_joints, 10)

        self.cli_load = self.create_client(LoadScenario, '/planning/load_scenario')
        self.cli_clean = self.create_client(Call, '/xarm/clean_error')

        # Wait for required services
        while not self.cli_load.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /planning/load_scenario...")
        while not self.cli_clean.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /xarm/clean_error...")

        self.get_logger().info("Services Connected. Starting Evaluation Loop...")
        self.timer = self.create_timer(2.0, self._run_next_scenario)

    def _load_scenarios(self):
        path = self.get_parameter('scenario_config_path').value
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Could not load scenarios from {path}: {e}")
            return []

    def _cb_robot_status(self, msg):
        """Captures Cartesian pose from the simulator robot status."""
        if len(msg.pose) >= 6:
            self.latest_ee_pose = np.array(msg.pose[:6])

    def _cb_joints(self, msg):
        """Records both Joint and Cartesian data when a measurement is active."""
        if self.is_recording and self.current_metrics is not None:
            pos = np.array(msg.position[:7])
            now = time.time()
            self.current_metrics.add_data_point(
                now, pos, self.latest_ee_pose, self.latest_latency, self.latest_clearance
            )

    def _cb_telemetry(self, msg):
        if len(msg.data) >= 2:
            self.latest_latency = msg.data[0]
            self.latest_clearance = msg.data[1]

    def _cb_status(self, msg):
        """Handles transitions between Resetting (Home) and Active Evaluation."""
        if msg.data is True:
            if self.is_resetting:
                self.get_logger().info("Home reached. Starting actual scenario...")
                self.is_resetting = False
                self._start_actual_evaluation()
            elif self.scenario_active:
                self.get_logger().info(f"Scenario {self.current_scenario_id} COMPLETED.")
                self._finish_scenario(success=True)

    def _cb_target_joints(self, msg):
        """Captures the target joint configuration from the planner/scene."""
        if len(msg.data) >= 7:
            self.latest_target_joints = np.array(msg.data[:7])

    def _run_next_scenario(self):
        """Step 1: Initiate reset by clearing obstacles and setting target to Home."""
        self.timer.cancel()
        self.current_run_index += 1

        if self.current_run_index >= self.limit:
            self._save_report()
            rclpy.shutdown()
            return

        self.current_scenario_id = self.scenarios_to_execute[self.current_run_index]
        self.get_logger().info(f"--- RUN {self.current_run_index + 1}/{self.limit} (S{self.current_scenario_id}) ---")

        # Call Scenario -1 to clear obstacles and set Home target in planner
        req = LoadScenario.Request()
        req.scenario_index = -1
        future = self.cli_load.call_async(req)
        future.add_done_callback(self._on_reset_scene_done)

    def _on_reset_scene_done(self, future):
        """Step 2: Unlock the robot now that obstacles are gone."""
        self.get_logger().info("Scene cleared. Unlocking robot...")
        clean_req = Call.Request()
        future_clean = self.cli_clean.call_async(clean_req)
        future_clean.add_done_callback(self._on_robot_unlocked)

    def _on_robot_unlocked(self, future):
        """Step 3: Flag that we are waiting for the arm to reach Home."""
        self.get_logger().info("Robot unlocked. Resetting to Home...")
        self.is_resetting = True
        # Watchdog for reset phase
        self.reset_start_time = time.time()
        self.reset_timer = self.create_timer(1.0, self._check_reset_timeout)

    def _check_reset_timeout(self):
        if self.is_resetting and (time.time() - self.reset_start_time > 15.0):
            self.get_logger().warn("Reset to Home timed out! Attempting to proceed...")
            self.is_resetting = False
            self.reset_timer.cancel()
            self._start_actual_evaluation()
        elif not self.is_resetting:
            self.reset_timer.cancel()

    def _start_actual_evaluation(self):
        """Step 4: Load the real scenario and begin performance recording."""
        self.current_metrics = TrajectoryMetrics()
        self.latest_latency = 0.0
        self.latest_clearance = 10.0

        # Load Actual Scenario
        req = LoadScenario.Request()
        req.scenario_index = self.current_scenario_id
        self.cli_load.call_async(req)

        self.is_recording = True
        self.scenario_active = True
        self.start_time = time.time()
        self.timeout_timer = self.create_timer(30.0, self._check_timeout)

    def _check_timeout(self):
        if self.scenario_active:
            if time.time() - self.start_time > 30.0:
                self.get_logger().warn(f"Scenario {self.current_scenario_id} TIMEOUT.")
                self._finish_scenario(success=False)

    def _finish_scenario(self, success):
        self.scenario_active = False
        self.is_recording = False
        if hasattr(self, 'timeout_timer'):
            self.timeout_timer.cancel()
            self.timeout_timer.destroy()

        if self.current_metrics:
            # Find the target_pose from loaded scenarios
            target = None
            if self.current_scenario_id < len(self.scenarios_data):
                target = np.array(self.scenarios_data[self.current_scenario_id].get('target_pose'))

            data = self.current_metrics.compute_all(
                target_pose=target,
                target_joints=self.latest_target_joints
            )

            if data:
                data['scenario_id'] = self.current_scenario_id
                data['success'] = success
                self.results.append(data)
                self.get_logger().info(
                    f"Result S{self.current_scenario_id}: Success={success}, "
                    f"Cartesian Error={data['final_distance_error']:.4f}m, "
                    f"Joint Error={data['final_joint_error']:.4f}rad"
                )

        # Wait 2 seconds before next reset sequence
        self.timer = self.create_timer(2.0, self._run_next_scenario)

    def _save_report(self):
        self.get_logger().info("Evaluation Finished. Saving report...")
        with open('evaluation_result-0.1-joint_space.json', 'w') as f:
            json.dump(self.results, f, indent=4)
        print("Report Saved.")


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()