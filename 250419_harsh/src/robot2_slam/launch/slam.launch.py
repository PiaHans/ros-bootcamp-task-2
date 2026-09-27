import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot2_slam = get_package_share_directory('robot2_slam')
    default_params_file = os.path.join(pkg_robot2_slam, 'config', 'slam_toolbox.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to slam_toolbox params YAML file'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot2',
        parameters=[
            default_params_file,
            {'use_sim_time': True}
        ],
        remappings=[
            ('map', '/robot2/map'),
            ('map_metadata', '/robot2/map_metadata')
        ],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        slam_node
    ])
