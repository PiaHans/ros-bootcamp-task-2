import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot2_slam = get_package_share_directory('robot2_slam')
    slam_config_file = os.path.join(pkg_robot2_slam, 'config', 'slam.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot2',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/map', '/robot2/map'),
            ('/map_metadata', '/robot2/map_metadata'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        slam_node
    ])
