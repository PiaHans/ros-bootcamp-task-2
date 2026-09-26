import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': f'{namespace}/odom' if namespace else 'odom',
            'base_frame': f'{namespace}/base_footprint' if namespace else 'base_footprint',
            'map_frame': f'{namespace}/map' if namespace else 'map',
            'scan_topic': f'/{namespace}/scan' if namespace else '/scan',
            'mode': 'mapping',
            'map_update_interval': 0.5,
            'resolution': 0.05,
            'max_laser_range': 3.5,
            'minimum_time_interval': 0.2,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True,
            'minimum_travel_distance': 0.10,
            'minimum_travel_heading': 0.10,
            'transform_publish_period': 0.02
        }],
        remappings=[
            ('/map', f'/{namespace}/map' if namespace else '/map'),
            ('/map_metadata', f'/{namespace}/map_metadata' if namespace else '/map_metadata'),
        ],
        output='screen'
    )

    nodes = [slam_node]

    if use_rviz:
        pkg_slam = get_package_share_directory('slam')
        rviz_config = os.path.join(pkg_slam, 'rviz', 'slam.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
        nodes.append(rviz_node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1', description='Robot namespace for SLAM scoping'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Open RViz to visualize live map'),
        OpaqueFunction(function=launch_setup)
    ])

