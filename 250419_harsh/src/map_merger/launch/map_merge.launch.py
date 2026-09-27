from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    r1_map_topic_arg = DeclareLaunchArgument(
        'robot1_map_topic',
        default_value='/robot1/map',
        description='Topic name for Robot 1 map'
    )

    r2_map_topic_arg = DeclareLaunchArgument(
        'robot2_map_topic',
        default_value='/robot2/map',
        description='Topic name for Robot 2 map'
    )

    merged_map_topic_arg = DeclareLaunchArgument(
        'merged_map_topic',
        default_value='/map',
        description='Topic name for output merged map'
    )

    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='map',
        description='Target frame_id for the merged map'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Map merge publication frequency in Hz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    merger_node = Node(
        package='map_merger',
        executable='map_merger_node',
        name='map_merger_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot1_map_topic': LaunchConfiguration('robot1_map_topic'),
            'robot2_map_topic': LaunchConfiguration('robot2_map_topic'),
            'merged_map_topic': LaunchConfiguration('merged_map_topic'),
            'output_frame': LaunchConfiguration('output_frame'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
    )

    return LaunchDescription([
        r1_map_topic_arg,
        r2_map_topic_arg,
        merged_map_topic_arg,
        output_frame_arg,
        publish_rate_arg,
        use_sim_time_arg,
        merger_node
    ])
