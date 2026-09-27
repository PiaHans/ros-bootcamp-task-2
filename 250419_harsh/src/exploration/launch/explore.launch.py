import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Namespace of the robot to explore with (robot1 or robot2)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    min_frontier_size_arg = DeclareLaunchArgument(
        'min_frontier_size',
        default_value='4',
        description='Minimum number of cells to constitute a valid frontier cluster'
    )

    min_goal_distance_arg = DeclareLaunchArgument(
        'min_goal_distance',
        default_value='0.4',
        description='Minimum distance (m) from robot to frontier candidate'
    )

    min_obstacle_clearance_arg = DeclareLaunchArgument(
        'min_obstacle_clearance',
        default_value='0.28',
        description='Minimum distance (m) from frontier candidate to nearest obstacle'
    )

    frontier_explorer_node = Node(
        package='exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'min_frontier_size': LaunchConfiguration('min_frontier_size'),
            'min_goal_distance': LaunchConfiguration('min_goal_distance'),
            'min_obstacle_clearance': LaunchConfiguration('min_obstacle_clearance'),
        }],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        min_frontier_size_arg,
        min_goal_distance_arg,
        min_obstacle_clearance_arg,
        frontier_explorer_node
    ])
