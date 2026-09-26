import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')
    pkg_slam = get_package_share_directory('slam')
    pkg_map_merger = get_package_share_directory('map_merger')
    pkg_bringup = get_package_share_directory('bringup')
    pkg_exploration = get_package_share_directory('exploration')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open unified RViz display for both robots and merged map'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value=LaunchConfiguration('rviz'),
        description='Alias for rviz'
    )

    autonomous_arg = DeclareLaunchArgument(
        'autonomous',
        default_value='true',
        description='Immediately start autonomous exploration on both robots'
    )

    # 1. Gazebo simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'simulation.launch.py')
        )
    )

    # 2. Spawn Robot 1 (Left arena: x=-2.0, y=0.0)
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot1',
            'x_pose': '-2.0',
            'y_pose': '0.0',
            'yaw': '0.0'
        }.items()
    )

    # 3. Spawn Robot 2 (Right arena: x=2.0, y=0.0)
    spawn_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot2',
            'x_pose': '2.0',
            'y_pose': '0.0',
            'yaw': '0.0'
        }.items()
    )

    # 4. SLAM for Robot 1 (map frame: robot1/map, scan: /robot1/scan)
    slam_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'namespace': 'robot1',
            'use_rviz': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 5. SLAM for Robot 2 (map frame: robot2/map, scan: /robot2/scan)
    slam_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'namespace': 'robot2',
            'use_rviz': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 6. Map Merger (fuses /robot1/map + /robot2/map -> /map)
    map_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_map_merger, 'launch', 'map_merger.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 7. Unified Multi-Robot RViz
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'multi_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # 8. Optional Autonomous Exploration for both robots
    multi_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_exploration, 'launch', 'multi_exploration.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(LaunchConfiguration('autonomous'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        use_rviz_arg,
        autonomous_arg,
        sim_launch,
        spawn_robot1,
        spawn_robot2,
        slam_robot1,
        slam_robot2,
        map_merger_launch,
        rviz_node,
        multi_exploration
    ])
