import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')
    pkg_slam = get_package_share_directory('slam')

    # Arguments
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot1', description='Robot namespace')
    x_arg = DeclareLaunchArgument('x_pose', default_value='-2.0', description='Initial X position')
    y_arg = DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial Y position')
    enable_slam_arg = DeclareLaunchArgument('enable_slam', default_value='false', description='Start SLAM automatically')

    # 1. Start Gazebo Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'simulation.launch.py')
        )
    )

    # 2. Spawn Robot 1
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    # 3. Parameterized SLAM for Robot 1 (optional via argument)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('robot_name')
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_slam'))
    )

    return LaunchDescription([
        robot_name_arg,
        x_arg,
        y_arg,
        enable_slam_arg,
        sim_launch,
        spawn_robot1,
        slam_launch
    ])

