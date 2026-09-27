import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot1_slam = get_package_share_directory('robot1_slam')
    params_file = os.path.join(pkg_robot1_slam, 'config', 'nav2.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    tf_remappings = [
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
        ('cmd_vel', '/robot1/cmd_vel')
    ]

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='robot1',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=tf_remappings
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='robot1',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='robot1',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=tf_remappings
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='robot1',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        controller_node,
        planner_node,
        behavior_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])
