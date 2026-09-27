import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory('simulation')

    world_path = os.path.join(pkg_simulation, 'worlds', 'maze.world')

    robot1_x = LaunchConfiguration('robot1_x', default='-3.0')
    robot1_y = LaunchConfiguration('robot1_y', default='-3.0')
    robot1_yaw = LaunchConfiguration('robot1_yaw', default='0.0')

    robot2_x = LaunchConfiguration('robot2_x', default='3.0')
    robot2_y = LaunchConfiguration('robot2_y', default='3.0')
    robot2_yaw = LaunchConfiguration('robot2_yaw', default='3.14159')

    # 1. Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Gazebo Client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 3. Spawn Robot 1 at (-3.0, -3.0)
    spawn_robot1_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
                ),
                launch_arguments={
                    'robot_name': 'robot1',
                    'x_pose': robot1_x,
                    'y_pose': robot1_y,
                    'yaw': robot1_yaw
                }.items()
            )
        ]
    )

    # 4. Spawn Robot 2 at (3.0, 3.0)
    spawn_robot2_cmd = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
                ),
                launch_arguments={
                    'robot_name': 'robot2',
                    'x_pose': robot2_x,
                    'y_pose': robot2_y,
                    'yaw': robot2_yaw
                }.items()
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot1_x', default_value='-3.0', description='Robot 1 initial X'),
        DeclareLaunchArgument('robot1_y', default_value='-3.0', description='Robot 1 initial Y'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0', description='Robot 1 initial Yaw'),

        DeclareLaunchArgument('robot2_x', default_value='3.0', description='Robot 2 initial X'),
        DeclareLaunchArgument('robot2_y', default_value='3.0', description='Robot 2 initial Y'),
        DeclareLaunchArgument('robot2_yaw', default_value='3.14159', description='Robot 2 initial Yaw'),

        gzserver_cmd,
        gzclient_cmd,
        spawn_robot1_cmd,
        spawn_robot2_cmd
    ])
