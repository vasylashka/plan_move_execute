import rclpy
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.vortex_strategy import VortexAPFStrategy

class VortexAPFPlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('vortex_apf_planner')

        self.declare_parameter('attraction_gain', 1.5)
        self.declare_parameter('repulsion_gain', 0.01)
        self.declare_parameter('influence_distance', 0.05)
        self.declare_parameter('vortex_gain', 0.85)
        self.declare_parameter('xi_max_multiplier', 3.0)

        self.strategy = VortexAPFStrategy(
            xi=self.get_parameter('attraction_gain').value,
            eta=self.get_parameter('repulsion_gain').value,
            rho0=self.get_parameter('influence_distance').value,
            gamma=self.get_parameter('vortex_gain').value,
            xi_max_multiplier=self.get_parameter('xi_max_multiplier').value
        )

        self.get_logger().info("Vortex APF Planner with Adaptive Attraction Ready.")

def main(args=None):
    rclpy.init(args=args)
    node = VortexAPFPlanner()
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