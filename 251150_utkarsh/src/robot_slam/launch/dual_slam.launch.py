import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_slam = get_package_share_directory('robot_slam')

    slam_config_r1 = os.path.join(pkg_robot_slam, 'config', 'slam_robot1.yaml')
    slam_config_r2 = os.path.join(pkg_robot_slam, 'config', 'slam_robot2.yaml')

    node_slam_robot1 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='robot1',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[slam_config_r1, {'use_sim_time': True}],
        remappings=[
            ('/map', '/robot1/map'),
            ('/map_metadata', '/robot1/map_metadata')
        ]
    )

    node_slam_robot2 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='robot2',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[slam_config_r2, {'use_sim_time': True}],
        remappings=[
            ('/map', '/robot2/map'),
            ('/map_metadata', '/robot2/map_metadata')
        ]
    )

    return LaunchDescription([
        node_slam_robot1,
        node_slam_robot2
    ])
