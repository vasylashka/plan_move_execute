from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # === Launch Arguments ===
    # This allows you to switch planners via CLI:
    # ros2 launch xarm_planner_evaluation evaluation.launch.py planner_type:=my_custom_planner
    planner_type_arg = DeclareLaunchArgument(
        'planner_type',
        default_value='joint_space_apf_planner',
        description='Name of the planner node executable to launch'
    )

    scenarios_to_run_arg = DeclareLaunchArgument(
        'scenarios_to_run',
        default_value='5',
        description='Number of scenarios to execute'
    )

    random_selection_arg = DeclareLaunchArgument(
        'random_selection',
        default_value='False',
        description='Whether to pick scenarios randomly instead of sequentially'
    )

    total_scenarios_arg = DeclareLaunchArgument(
        'total_scenarios',
        default_value='100',
        description='Total number of scenarios generated in the JSON'
    )

    # === Node Configurations ===

    # 1. Physics Simulator (The "Real" Robot)
    sim_node = Node(
        package='xarm_sim_api',
        executable='xarm7_physics_sim',
        name='xarm_sim',
        output='screen',
        parameters=[{'frequency': 240.0}]
    )

    # 2. Planning Scene (The "World" & Collision Checking)
    scene_node = Node(
        package='xarm_local_planner',
        executable='xarm_planning_scene',
        name='xarm_planning_scene',
        output='screen'
    )

    # 3. The Planner Under Test (Selected via Argument)
    # We map the argument directly to the executable name
    planner_node = Node(
        package='xarm_local_planner',
        executable=LaunchConfiguration('planner_type'),
        name='local_planner_under_test',
        output='screen',
        parameters=[{'step_size': 0.03}]
    )

    # 4. The Evaluator (Test Orchestrator)
    evaluator_node = Node(
        package='xarm_planner_evaluation',
        executable='evaluator_node',
        name='evaluator_node',
        output='screen',
        parameters=[{
            'scenarios_to_run': LaunchConfiguration('scenarios_to_run'),
            'random_selection': LaunchConfiguration('random_selection'),
            'total_scenarios': LaunchConfiguration('total_scenarios')
        }]
    )

    return LaunchDescription([
        planner_type_arg,
        sim_node,
        scene_node,
        planner_node,
        evaluator_node
    ])