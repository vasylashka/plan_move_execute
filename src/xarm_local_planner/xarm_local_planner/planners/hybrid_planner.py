import rclpy
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.hybrid_strategy import HybridVortexVOStrategy


class HybridVortexVOPlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('hybrid_vortex_vo_planner')

        # Core APF parameters
        self.declare_parameter('attraction_gain', 2.5)
        self.declare_parameter('repulsion_gain', 0.01)
        self.declare_parameter('influence_distance', 0.03)

        # Virtual Obstacle parameters
        self.declare_parameter('vo_repulsion_gain', 0.5)
        self.declare_parameter('vo_influence_dist', 1.5)
        self.declare_parameter('goal_tolerance', 0.12)

        # Guided Vortex & Adaptive parameters
        self.declare_parameter('epsilon', 0.8)
        self.declare_parameter('xi_max_multiplier', 3.0)

        # Initialize the strategy without joint weighting
        self.strategy = HybridVortexVOStrategy(
            xi=self.get_parameter('attraction_gain').value,
            eta=self.get_parameter('repulsion_gain').value,
            rho0=self.get_parameter('influence_distance').value,
            vo_eta=self.get_parameter('vo_repulsion_gain').value,
            vo_rho0=self.get_parameter('vo_influence_dist').value,
            epsilon=self.get_parameter('epsilon').value,
            xi_max_multiplier=self.get_parameter('xi_max_multiplier').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value
        )

        self.get_logger().info("Hybrid Planner (Vortex + VO + Adaptive) Ready.")


def main(args=None):
    rclpy.init(args=args)
    node = HybridVortexVOPlanner()
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