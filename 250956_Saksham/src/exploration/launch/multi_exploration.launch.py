import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_exploration = get_package_share_directory('exploration')
    nav_launch_file = os.path.join(pkg_exploration, 'launch', 'nav2_robot.launch.py')

    config_robot1 = os.path.join(pkg_exploration, 'config', 'nav2_robot1.yaml')
    config_robot2 = os.path.join(pkg_exploration, 'config', 'nav2_robot2.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack for each robot'
    )

    # 1. Nav2 Navigation Stack for Robot 1
    nav2_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file),
        launch_arguments={
            'namespace': 'robot1',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': config_robot1
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_nav2'))
    )

    # 2. Nav2 Navigation Stack for Robot 2
    nav2_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file),
        launch_arguments={
            'namespace': 'robot2',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': config_robot2
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_nav2'))
    )

    # 3. Frontier Exploration for Robot 1 (Directs to center of nearest frontier via Nav2)
    exploration_robot1 = Node(
        package='exploration',
        executable='frontier_exploration',
        name='frontier_exploration',
        namespace='robot1',
        parameters=[{
            'robot_namespace': 'robot1',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'min_frontier_size': 3,
            'linear_speed': 0.22,
            'angular_speed': 0.60,
            'obstacle_dist': 0.65,
            'goal_lock_duration': 6.0,
            'stuck_timeout': 7.0,
            'stuck_dist_threshold': 0.05
        }],
        output='screen'
    )

    # 4. Frontier Exploration for Robot 2 (Directs to center of nearest frontier via Nav2)
    exploration_robot2 = Node(
        package='exploration',
        executable='frontier_exploration',
        name='frontier_exploration',
        namespace='robot2',
        parameters=[{
            'robot_namespace': 'robot2',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'min_frontier_size': 3,
            'linear_speed': 0.22,
            'angular_speed': 0.60,
            'obstacle_dist': 0.65,
            'goal_lock_duration': 6.0,
            'stuck_timeout': 7.0,
            'stuck_dist_threshold': 0.05
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_nav2_arg,
        nav2_robot1,
        nav2_robot2,
        exploration_robot1,
        exploration_robot2
    ])
