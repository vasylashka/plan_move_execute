import rclpy
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.pso_strategy import PsoOptimizedStrategy

class PsoOptimizedPlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('pso_optimized_planner')

        self.declare_parameter('influence_distance', 0.1)
        self.declare_parameter('pso_particles', 15)
        self.declare_parameter('pso_iterations', 5)

        self.strategy = PsoOptimizedStrategy(
            rho0=self.get_parameter('influence_distance').value,
            num_particles=self.get_parameter('pso_particles').value,
            iterations=self.get_parameter('pso_iterations').value
        )

        self.get_logger().info("PSO-Optimized APF Planner initialized. Adaptive gains active.")

def main(args=None):
    rclpy.init(args=args)
    node = PsoOptimizedPlanner()
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