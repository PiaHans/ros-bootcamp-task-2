import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot1_x = LaunchConfiguration('robot1_x', default='-1.5')
    robot1_y = LaunchConfiguration('robot1_y', default='0.5')
    robot2_x = LaunchConfiguration('robot2_x', default='1.5')
    robot2_y = LaunchConfiguration('robot2_y', default='-0.5')

    merger_node = Node(
        package='map_merger',
        executable='map_merger_node',
        name='map_merger',
        output='screen',
        parameters=[{
            'robot1_x': robot1_x,
            'robot1_y': robot1_y,
            'robot2_x': robot2_x,
            'robot2_y': robot2_y,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot1_x', default_value='-1.5'),
        DeclareLaunchArgument('robot1_y', default_value='0.5'),
        DeclareLaunchArgument('robot2_x', default_value='1.5'),
        DeclareLaunchArgument('robot2_y', default_value='-0.5'),
        merger_node
    ])
