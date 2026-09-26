import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def create_nav_nodes(robot_name, nav_config):
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=robot_name,
            output='screen',
            parameters=[nav_config],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=robot_name,
            output='screen',
            parameters=[nav_config]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=robot_name,
            output='screen',
            parameters=[nav_config]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=robot_name,
            output='screen',
            parameters=[nav_config]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=robot_name,
            output='screen',
            parameters=[nav_config]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=robot_name,
            output='screen',
            parameters=[nav_config]
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=robot_name,
            output='screen',
            parameters=[nav_config],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': lifecycle_nodes
            }]
        )
    ]

def generate_launch_description():
    pkg_navigation = get_package_share_directory('navigation')
    nav_config_r1 = os.path.join(pkg_navigation, 'config', 'nav2_robot1.yaml')
    nav_config_r2 = os.path.join(pkg_navigation, 'config', 'nav2_robot2.yaml')

    nodes = []
    nodes.extend(create_nav_nodes('robot1', nav_config_r1))
    nodes.extend(create_nav_nodes('robot2', nav_config_r2))

    return LaunchDescription(nodes)
