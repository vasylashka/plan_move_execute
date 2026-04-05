import rclpy
from xarm_local_planner.planners.base_planner import BaseLocalPlanner
from xarm_local_planner.strategies.beetle_strategy import DefaultBeetleStrategy


class DefaultBeetlePlanner(BaseLocalPlanner):
    def __init__(self):
        super().__init__('default_beetle_planner')

        self.declare_parameter('attraction_gain', 3.0)
        self.declare_parameter('repulsion_gain', 0.01)
        self.declare_parameter('influence_distance', 0.05)

        # Параметри BAS та детекції
        self.declare_parameter('bas_antenna_dist', 0.3)
        self.declare_parameter('bas_recovery_steps', 60)
        self.declare_parameter('variance_threshold', 0.85)  # Наскільки чутливо реагувати на осциляції
        self.declare_parameter('goal_tolerance', 0.1)

        self.strategy = DefaultBeetleStrategy(
            xi=self.get_parameter('attraction_gain').value,
            eta=self.get_parameter('repulsion_gain').value,
            rho0=self.get_parameter('influence_distance').value,
            bas_d=self.get_parameter('bas_antenna_dist').value,
            bas_steps=self.get_parameter('bas_recovery_steps').value,
            variance_threshold=self.get_parameter('variance_threshold').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value
        )


def main(args=None):
    rclpy.init(args=args)
    node = DefaultBeetlePlanner()
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