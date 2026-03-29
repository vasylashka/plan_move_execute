import rclpy
from threading import Thread
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.weighted_strategy import WeightedAPFStrategy

class WeightedJointSpaceAPFPlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('weighted_joint_space_apf_planner')

        # Declare specific APF params
        self.declare_parameter('attraction_gain', 1.5)
        self.declare_parameter('repulsion_gain', 0.01)
        self.declare_parameter('influence_distance', 0.03)

        # Initialize the strategy
        self.strategy = WeightedAPFStrategy(
            xi=self.get_parameter('attraction_gain').value,
            eta=self.get_parameter('repulsion_gain').value,
            rho0=self.get_parameter('influence_distance').value
        )

        self.get_logger().info("Planner with Standard APF Strategy Ready.")


def main(args=None):
    rclpy.init(args=args)
    node = WeightedJointSpaceAPFPlanner()
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