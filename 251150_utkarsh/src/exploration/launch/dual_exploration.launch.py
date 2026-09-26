from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    explorer_r1 = Node(
        package='exploration',
        executable='frontier_explorer',
        name='frontier_explorer_r1',
        output='screen',
        parameters=[{
            'robot_name': 'robot1',
            'use_sim_time': True
        }]
    )

    explorer_r2 = Node(
        package='exploration',
        executable='frontier_explorer',
        name='frontier_explorer_r2',
        output='screen',
        parameters=[{
            'robot_name': 'robot2',
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        explorer_r1,
        explorer_r2
    ])
