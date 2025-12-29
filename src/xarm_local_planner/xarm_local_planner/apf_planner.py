import rclpy
from rclpy.node import Node
import numpy as np
import time
from threading import Thread

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from xarm_local_msgs.srv import GetApfDistances
from xarm_msgs.srv import MoveJoint


class JointSpaceAPFPlanner(Node):
    def __init__(self):
        super().__init__('joint_space_apf_planner')

        # Attraction Gain (Pull to goal)
        self.declare_parameter('attraction_gain', 1.5)
        # Repulsion Gain (Push from obstacles)
        self.declare_parameter('repulsion_gain', 0.05)
        # Influence Distance (Start avoiding when closer than this) - meters
        self.declare_parameter('influence_distance', 0.05)
        # Simulation Step Size (How fast the robot moves per tick)
        self.declare_parameter('step_size', 0.03)

        self.xi = self.get_parameter('attraction_gain').value
        self.eta = self.get_parameter('repulsion_gain').value
        self.rho0 = self.get_parameter('influence_distance').value
        self.alpha = self.get_parameter('step_size').value

        # Logging Parameters
        self.get_logger().info("========== APF PLANNER STARTED ==========")
        self.get_logger().info(f"Params -> Att: {self.xi}, Rep: {self.eta}, Dist: {self.rho0}, Step: {self.alpha}")

        self.current_joints = np.zeros(7)
        self.q_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.planning_active = False

        # State flags for logging
        self.goal_reached_flag = False

        self.sub_joints = self.create_subscription(
            JointState,
            '/xarm/joint_states',
            self.joint_callback,
            10
        )

        self.sub_goal = self.create_subscription(
            Float32MultiArray,
            '/planning/target_joints',
            self.goal_callback,
            10
        )

        self.cli_env = self.create_client(GetApfDistances, '/planning/get_apf_distances')
        self.cli_move = self.create_client(MoveJoint, '/xarm/set_servo_angle')

        self.get_logger().info("Waiting for services...")
        while not self.cli_env.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /planning/get_apf_distances...')
        while not self.cli_move.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /xarm/set_servo_angle...')

        self.get_logger().info("APF Planner Ready. Starting Control Loop...")

        self.planning_active = True
        self.plan_thread = Thread(target=self.control_loop)
        self.plan_thread.start()

    def joint_callback(self, msg):
        """Update current robot configuration"""
        if len(msg.position) >= 7:
            self.current_joints = np.array(msg.position[:7])

    def goal_callback(self, msg):
        """Update target goal from simulation"""
        if len(msg.data) >= 7:
            new_goal = np.array(msg.data[:7])
            # Only update and log if the goal has changed significantly (prevent spam)
            if np.linalg.norm(new_goal - self.q_goal) > 1e-3:
                self.q_goal = new_goal
                self.goal_reached_flag = False  # Reset reached state
                self.get_logger().info(f"New Goal Received: {np.round(self.q_goal, 3)}")

    def calculate_apf(self):
        """
        Calculates the Total Force = F_attraction + F_repulsion
        """
        error = self.current_joints - self.q_goal

        if np.linalg.norm(error) < 0.01:
            return None, True

        tau_att = -self.xi * error

        tau_rep = np.zeros(7)

        req = GetApfDistances.Request()
        future = self.cli_env.call_async(req)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 1.0:
                self.get_logger().warn("Service timeout: /planning/get_apf_distances")
                return None, False
            time.sleep(0.01)

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None, False

        if resp and resp.data:
            for link_data in resp.data:
                dist = link_data.min_distance

                if 0 < dist < self.rho0:
                    # LOGGING: Throttled warning when actively avoiding obstacles
                    self.get_logger().warn(
                        f"Avoiding Obstacle! Link: {link_data.link_name}, Dist: {dist:.3f}m",
                        throttle_duration_sec=2.0
                    )

                    mag = self.eta * (1.0 / dist - 1.0 / self.rho0) * (1.0 / (dist ** 2))

                    dir_vec = np.array([
                        link_data.direction_vector.x,
                        link_data.direction_vector.y,
                        link_data.direction_vector.z
                    ])

                    norm = np.linalg.norm(dir_vec)
                    if norm > 1e-6:
                        dir_vec = dir_vec / norm

                    F_cart = -1.0 * mag * dir_vec

                    full_jac = np.array(link_data.jacobian)
                    if len(full_jac) > 0:
                        cols = len(full_jac) // 3
                        J = full_jac.reshape(3, cols)
                        J_arm = J[:, :7]  # Slice for arm joints

                        tau_link = np.dot(J_arm.T, F_cart)
                        tau_rep += tau_link

        return tau_att + tau_rep, False

    def control_loop(self):
        time.sleep(2.0)  # Warmup
        self.get_logger().info("Control Loop Active.")

        while rclpy.ok() and self.planning_active:
            torque, reached = self.calculate_apf()

            if reached:
                if not self.goal_reached_flag:
                    self.get_logger().info("Target Reached! Holding position.")
                    self.goal_reached_flag = True
                time.sleep(0.1)
                continue

            if torque is None:
                time.sleep(0.1)
                continue

            # Reset flag if we are moving again
            if self.goal_reached_flag:
                self.goal_reached_flag = False

            torque = np.clip(torque, -1.0, 1.0)

            step = torque * self.alpha
            q_next = self.current_joints + step

            self.send_command(q_next)

            time.sleep(0.05)

    def send_command(self, q_target):
        req = MoveJoint.Request()
        req.angles = q_target.tolist()
        req.speed = 0.0
        req.wait = False

        self.cli_move.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = JointSpaceAPFPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down APF Planner...")
        node.planning_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()