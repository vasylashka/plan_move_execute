import rclpy
from rclpy.node import Node
import time
import json
import os
import random  # Added for random selection
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from xarm_local_msgs.srv import LoadScenario
from xarm_msgs.srv import Call
from xarm_planner_evaluation.metrics import TrajectoryMetrics


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator_node')
        # Print immediately to confirm startup
        print(">>> EVALUATOR NODE STARTED <<<")
        self.get_logger().info("Evaluator Node Initialized. Waiting for services...")

        # Declare parameters
        self.declare_parameter('scenarios_to_run', 5)
        self.declare_parameter('random_selection', False)
        self.declare_parameter('total_scenarios', 100)

        self.limit = self.get_parameter('scenarios_to_run').value
        self.is_random = self.get_parameter('random_selection').value
        self.total_scenarios = self.get_parameter('total_scenarios').value

        # Generate the list of scenarios to run
        if self.is_random:
            # Ensure we don't try to sample more scenarios than exist
            sample_size = min(self.limit, self.total_scenarios)
            self.scenarios_to_execute = random.sample(range(self.total_scenarios), sample_size)
            self.limit = sample_size
            self.get_logger().info(f"Random mode ON. Selected scenarios: {self.scenarios_to_execute}")
        else:
            self.scenarios_to_execute = list(range(self.limit))
            self.get_logger().info(f"Sequential mode ON. Running first {self.limit} scenarios.")

        self.current_run_index = -1
        self.current_scenario_id = -1

        self.current_metrics = None
        self.results = []
        self.is_recording = False
        self.scenario_active = False
        self.latest_latency = 0.0
        self.latest_clearance = 10.0

        # ROS Interfaces
        self.sub_joints = self.create_subscription(
            JointState, '/xarm/joint_states', self._cb_joints, 10)
        self.sub_telemetry = self.create_subscription(
            Float32MultiArray, '/planning/telemetry', self._cb_telemetry, 10)
        self.sub_status = self.create_subscription(
            Bool, '/planning/status_reached', self._cb_status, 10)

        self.cli_load = self.create_client(LoadScenario, '/planning/load_scenario')
        self.cli_clean = self.create_client(Call, '/xarm/clean_error')

        # Wait for service
        while not self.cli_load.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /planning/load_scenario...")

        self.get_logger().info("Services Connected. Starting Evaluation Loop...")
        self.timer = self.create_timer(2.0, self._run_next_scenario)

    def _cb_joints(self, msg):
        if self.is_recording and self.current_metrics is not None:
            pos = np.array(msg.position[:7])
            now = time.time()
            self.current_metrics.add_data_point(
                now, pos, self.latest_latency, self.latest_clearance
            )

    def _cb_telemetry(self, msg):
        if len(msg.data) >= 2:
            self.latest_latency = msg.data[0]
            self.latest_clearance = msg.data[1]

    def _cb_status(self, msg):
        if self.scenario_active and msg.data == True:
            self.get_logger().info(f"Scenario {self.current_scenario_id} COMPLETED.")
            self._finish_scenario(success=True)

    def _run_next_scenario(self):
        self.timer.cancel()
        self.current_run_index += 1

        # Check if we have completed all requested runs
        if self.current_run_index >= self.limit:
            self._save_report()
            rclpy.shutdown()
            return

        # Get the actual scenario ID from our generated list
        self.current_scenario_id = self.scenarios_to_execute[self.current_run_index]

        self.get_logger().info(f"--- STARTING RUN {self.current_run_index + 1}/{self.limit} (SCENARIO ID: {self.current_scenario_id}) ---")

        # Call clean error just in case
        clean_req = Call.Request()
        self.cli_clean.call_async(clean_req)
        time.sleep(0.5)

        # Setup Metrics
        self.current_metrics = TrajectoryMetrics()
        self.latest_latency = 0.0
        self.latest_clearance = 10.0

        # Load Scenario
        req = LoadScenario.Request()
        req.scenario_index = self.current_scenario_id
        future = self.cli_load.call_async(req)

        # Enable Recording
        self.is_recording = True
        self.scenario_active = True
        self.start_time = time.time()

        # Timeout Watchdog
        self.timeout_timer = self.create_timer(10.0, self._check_timeout)

    def _check_timeout(self):
        if self.scenario_active:
            if time.time() - self.start_time > 15.0:
                self.get_logger().warn(f"Scenario {self.current_scenario_id} TIMEOUT.")
                self._finish_scenario(success=False)

    def _finish_scenario(self, success):
        self.scenario_active = False
        self.is_recording = False
        if hasattr(self, 'timeout_timer'):
            self.timeout_timer.cancel()
            self.timeout_timer.destroy()

        if self.current_metrics:
            data = self.current_metrics.compute_all()
            if data:
                data['scenario_id'] = self.current_scenario_id
                data['success'] = success
                self.results.append(data)
                print(f"Result S{self.current_scenario_id}: {data}")

        self.timer = self.create_timer(2.0, self._run_next_scenario)

    def _save_report(self):
        print("\n========= EVALUATION REPORT =========")
        with open('evaluation_results.json', 'w') as f:
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
        # Make sure rclpy is still okay to shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()