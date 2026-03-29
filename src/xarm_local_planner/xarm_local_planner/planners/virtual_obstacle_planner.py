import rclpy
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.virtual_obstacle_strategy import VirtualObstacleAPFStrategy


class VirtualObstacleAPFPlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('virtual_obstacle_apf_planner')

        # Standard APF params
        self.declare_parameter('attraction_gain', 1.5)
        self.declare_parameter('repulsion_gain', 0.01)
        self.declare_parameter('influence_distance', 0.03)

        # Virtual Obstacle specific params
        self.declare_parameter('vo_repulsion_gain', 0.1)
        self.declare_parameter('vo_influence_dist', 1.0)
        self.declare_parameter('vo_stuck_window', 50)
        self.declare_parameter('vo_stuck_threshold', 0.05)
        self.declare_parameter('vo_oscillation_ratio', 3.0)
        self.declare_parameter('goal_tolerance', 0.1)  # NEW PARAMETER

        # Initialize the strategy
        self.strategy = VirtualObstacleAPFStrategy(
            xi=self.get_parameter('attraction_gain').value,
            eta=self.get_parameter('repulsion_gain').value,
            rho0=self.get_parameter('influence_distance').value,
            vo_eta=self.get_parameter('vo_repulsion_gain').value,
            vo_rho0=self.get_parameter('vo_influence_dist').value,
            stuck_window=self.get_parameter('vo_stuck_window').value,
            stuck_threshold=self.get_parameter('vo_stuck_threshold').value,
            oscillation_ratio=self.get_parameter('vo_oscillation_ratio').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value  # NEW PARAMETER
        )

        self.get_logger().info("Planner with Virtual Obstacles Strategy Ready.")


def main(args=None):
    rclpy.init(args=args)
    node = VirtualObstacleAPFPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.planning_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()