import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    autonomous_arg = DeclareLaunchArgument(
        'autonomous',
        default_value='true',
        description='Launch Nav2 and autonomous frontier exploration'
    )

    # Master launch includes multi_robot with all tasks enabled
    all_tasks_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'multi_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
            'autonomous': LaunchConfiguration('autonomous')
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        autonomous_arg,
        all_tasks_launch
    ])
