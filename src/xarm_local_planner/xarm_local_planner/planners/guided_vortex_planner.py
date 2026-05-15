import rclpy
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.guided_vortex_strategy import GuidedVortexStrategy

class GuidedVortexPlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('guided_vortex_planner')

        self.declare_parameter('attraction_gain', 1.5)
        self.declare_parameter('repulsion_gain', 0.01)
        self.declare_parameter('influence_distance', 0.05)
        self.declare_parameter('slide_gain', 0.8)
        self.declare_parameter('xi_max_multiplier', 3.0)
        self.declare_parameter('goal_tolerance', 0.1)

        self.strategy = GuidedVortexStrategy(
            xi=self.get_parameter('attraction_gain').value,
            eta=self.get_parameter('repulsion_gain').value,
            rho0=self.get_parameter('influence_distance').value,
            epsilon=self.get_parameter('slide_gain').value,
            xi_max_multiplier=self.get_parameter('xi_max_multiplier').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value
        )

        self.get_logger().info("Guided Vortex Planner initialized with configurable Goal Tolerance.")

def main(args=None):
    rclpy.init(args=args)
    node = GuidedVortexPlanner()
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