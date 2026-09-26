import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')
    pkg_slam = get_package_share_directory('robot_slam')
    pkg_nav = get_package_share_directory('navigation')
    pkg_merger = get_package_share_directory('map_merger')
    pkg_explore = get_package_share_directory('exploration')
    pkg_bringup = get_package_share_directory('bringup')

    rviz_config_path = os.path.join(pkg_bringup, 'rviz', 'master.rviz')

    robot1_x = LaunchConfiguration('robot1_x', default='-1.5')
    robot1_y = LaunchConfiguration('robot1_y', default='0.5')
    robot2_x = LaunchConfiguration('robot2_x', default='1.5')
    robot2_y = LaunchConfiguration('robot2_y', default='-0.5')

    # 1. Simulation (Gazebo + both robots)
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'robot1_x': robot1_x,
            'robot1_y': robot1_y,
            'robot2_x': robot2_x,
            'robot2_y': robot2_y
        }.items()
    )

    # 2. Dual SLAM (Delay 5s to ensure simulation is ready)
    launch_slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'dual_slam.launch.py')
                )
            )
        ]
    )

    # 3. Dual Nav2 Navigation (Delay 10s to ensure SLAM maps are publishing)
    launch_navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'dual_navigation.launch.py')
                )
            )
        ]
    )

    # 4. Map Merger (Delay 12s)
    launch_map_merger = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_merger, 'launch', 'map_merger.launch.py')
                ),
                launch_arguments={
                    'robot1_x': robot1_x,
                    'robot1_y': robot1_y,
                    'robot2_x': robot2_x,
                    'robot2_y': robot2_y
                }.items()
            )
        ]
    )

    # 5. Dual Exploration (Delay 16s to ensure Nav2 action servers are fully active)
    launch_exploration = TimerAction(
        period=16.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_explore, 'launch', 'dual_exploration.launch.py')
                )
            )
        ]
    )

    # 6. RViz2 Visualizer
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_master',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot1_x', default_value='-1.5'),
        DeclareLaunchArgument('robot1_y', default_value='0.5'),
        DeclareLaunchArgument('robot2_x', default_value='1.5'),
        DeclareLaunchArgument('robot2_y', default_value='-0.5'),
        launch_simulation,
        launch_slam,
        launch_navigation,
        launch_map_merger,
        launch_exploration,
        node_rviz
    ])
