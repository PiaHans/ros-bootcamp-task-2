from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    node = Node(
        package='map_merger',
        executable='map_merger',
        name='map_merger',
        parameters=[{
            'robot_namespaces': ['robot1', 'robot2'],
            'merged_frame_id': 'map',
            'merged_topic': '/map',
            'resolution': 0.05,
            'publish_rate': 2.0,
            'robot1_initial_pose': [-2.0, 0.0, 0.0],
            'robot2_initial_pose': [2.0, 0.0, 0.0],
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        node
    ])
