import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')
    pkg_sim = get_package_share_directory('simulation')
    pkg_r1_slam = get_package_share_directory('robot1_slam')
    pkg_r2_slam = get_package_share_directory('robot2_slam')
    pkg_exploration = get_package_share_directory('exploration')
    pkg_map_merger = get_package_share_directory('map_merger')

    rviz_config = os.path.join(pkg_bringup, 'rviz', 'multi_robot.rviz')

    # Arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')

    # 1. Gazebo Simulation with both robots
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'simulation.launch.py'))
    )

    # 2. SLAM Toolbox for Robot 1 and Robot 2 (delayed to allow Gazebo to spin up)
    slam_cmd = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_r1_slam, 'launch', 'slam.launch.py')),
                launch_arguments={'params_file': os.path.join(pkg_r1_slam, 'config', 'slam_toolbox.yaml')}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_r2_slam, 'launch', 'slam.launch.py')),
                launch_arguments={'params_file': os.path.join(pkg_r2_slam, 'config', 'slam_toolbox.yaml')}.items()
            ),
        ]
    )

    # 3. Nav2 Stacks for Robot 1 and Robot 2
    nav2_cmd = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'nav2_r1.launch.py'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'nav2_r2.launch.py'))
            ),
        ]
    )

    # 4. Map Merger node fusing both maps into /map
    map_merger_cmd = TimerAction(
        period=11.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_map_merger, 'launch', 'map_merge.launch.py'))
            )
        ]
    )

    # 5. Autonomous Frontier Explorers for Robot 1 and Robot 2
    exploration_cmd = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_exploration, 'launch', 'explore.launch.py')),
                launch_arguments={'robot_namespace': 'robot1'}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_exploration, 'launch', 'explore.launch.py')),
                launch_arguments={'robot_namespace': 'robot2'}.items()
            ),
        ]
    )

    # 6. RViz visualization with sanitized environment (prevents snap libpthread conflict)
    rviz_env = dict(os.environ)
    if 'LD_LIBRARY_PATH' in rviz_env:
        rviz_env['LD_LIBRARY_PATH'] = ':'.join(
            [p for p in rviz_env['LD_LIBRARY_PATH'].split(':') if 'snap' not in p]
        )
    if os.path.exists('/lib/x86_64-linux-gnu/libpthread.so.0'):
        rviz_env['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libpthread.so.0'

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        env=rviz_env,
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        sim_cmd,
        slam_cmd,
        nav2_cmd,
        map_merger_cmd,
        exploration_cmd,
        rviz_cmd
    ])
