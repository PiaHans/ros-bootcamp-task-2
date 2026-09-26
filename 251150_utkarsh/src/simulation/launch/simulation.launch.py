import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')

    r1_x = LaunchConfiguration('robot1_x', default='-1.5')
    r1_y = LaunchConfiguration('robot1_y', default='0.5')
    r1_yaw = LaunchConfiguration('robot1_yaw', default='0.0')

    r2_x = LaunchConfiguration('robot2_x', default='1.5')
    r2_y = LaunchConfiguration('robot2_y', default='-0.5')
    r2_yaw = LaunchConfiguration('robot2_yaw', default='3.1415')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'gazebo.launch.py')
        )
    )

    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot1',
            'x_pose': r1_x,
            'y_pose': r1_y,
            'yaw_pose': r1_yaw
        }.items()
    )

    spawn_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot2',
            'x_pose': r2_x,
            'y_pose': r2_y,
            'yaw_pose': r2_yaw
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot1_x', default_value='-1.5'),
        DeclareLaunchArgument('robot1_y', default_value='0.5'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot2_x', default_value='1.5'),
        DeclareLaunchArgument('robot2_y', default_value='-0.5'),
        DeclareLaunchArgument('robot2_yaw', default_value='3.1415'),
        gazebo_cmd,
        spawn_robot1,
        spawn_robot2
    ])
