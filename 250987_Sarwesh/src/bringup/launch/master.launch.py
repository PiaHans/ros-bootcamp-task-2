import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')
    pkg_robot1_slam = get_package_share_directory('robot1_slam')
    pkg_robot2_slam = get_package_share_directory('robot2_slam')
    pkg_exploration = get_package_share_directory('exploration')
    pkg_map_merger = get_package_share_directory('map_merger')
    pkg_bringup = get_package_share_directory('bringup')

    # Configurable initial coordinates
    robot1_x = LaunchConfiguration('robot1_x', default='-3.0')
    robot1_y = LaunchConfiguration('robot1_y', default='-3.0')
    robot1_yaw = LaunchConfiguration('robot1_yaw', default='0.0')

    robot2_x = LaunchConfiguration('robot2_x', default='3.0')
    robot2_y = LaunchConfiguration('robot2_y', default='3.0')
    robot2_yaw = LaunchConfiguration('robot2_yaw', default='3.14159')

    open_rviz = LaunchConfiguration('open_rviz', default='true')

    # 1. Gazebo Simulator & Dual Robot Spawning (t = 0s)
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'robot1_x': robot1_x, 'robot1_y': robot1_y, 'robot1_yaw': robot1_yaw,
            'robot2_x': robot2_x, 'robot2_y': robot2_y, 'robot2_yaw': robot2_yaw
        }.items()
    )

    # 2. Dual SLAM Pipelines (t = 6s)
    slam_cmd = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot1_slam, 'launch', 'slam.launch.py')
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot2_slam, 'launch', 'slam.launch.py')
                )
            )
        ]
    )

    # 3. Dual Nav2 Stacks (t = 11s)
    nav2_cmd = TimerAction(
        period=11.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot1_slam, 'launch', 'nav2.launch.py')
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot2_slam, 'launch', 'nav2.launch.py')
                )
            )
        ]
    )

    # 4. Map Merger Node (t = 14s)
    map_merger_cmd = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='map_merger',
                executable='map_merger_node',
                name='map_merger_node',
                output='screen',
                parameters=[{
                    'robot1_x': robot1_x,
                    'robot1_y': robot1_y,
                    'robot1_yaw': robot1_yaw,
                    'robot2_x': robot2_x,
                    'robot2_y': robot2_y,
                    'robot2_yaw': robot2_yaw
                }]
            )
        ]
    )

    # 5. Dual Autonomous Frontier Explorers (t = 16s)
    explore_cmd = TimerAction(
        period=16.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_exploration, 'launch', 'explore.launch.py')
                ),
                launch_arguments={'robot_name': 'robot1'}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_exploration, 'launch', 'explore.launch.py')
                ),
                launch_arguments={'robot_name': 'robot2'}.items()
            )
        ]
    )

    # 6. Multi-Robot RViz (t = 17s)
    rviz_config_file = os.path.join(pkg_bringup, 'config', 'multi_robot.rviz')
    rviz_cmd = TimerAction(
        period=17.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_multi_robot',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(open_rviz)
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

        DeclareLaunchArgument('open_rviz', default_value='true', description='Open multi-robot RViz'),

        sim_cmd,
        slam_cmd,
        nav2_cmd,
        map_merger_cmd,
        explore_cmd,
        rviz_cmd
    ])
