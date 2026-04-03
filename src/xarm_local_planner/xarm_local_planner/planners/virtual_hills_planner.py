import rclpy
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.virtual_hills_strategy import VirtualHillsAPFStrategy

class VirtualHillsAPFPlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('virtual_hills_apf_planner')

        # Базові параметри APF
        self.declare_parameter('attraction_gain', 1.5)
        self.declare_parameter('repulsion_gain', 0.01)
        self.declare_parameter('influence_distance', 0.03)

        # Параметри пагорбів
        self.declare_parameter('hill_height', 3.0)
        self.declare_parameter('hill_sigma', 0.15)
        self.declare_parameter('variance_threshold', 0.8)
        self.declare_parameter('goal_tolerance', 0.1)

        self.strategy = VirtualHillsAPFStrategy(
            xi=self.get_parameter('attraction_gain').value,
            eta=self.get_parameter('repulsion_gain').value,
            rho0=self.get_parameter('influence_distance').value,
            hill_height=self.get_parameter('hill_height').value,
            hill_sigma=self.get_parameter('hill_sigma').value,
            variance_threshold=self.get_parameter('variance_threshold').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value
        )

        self.get_logger().info("Virtual Hills APF planner is ready.")

def main(args=None):
    rclpy.init(args=args)
    node = VirtualHillsAPFPlanner()
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