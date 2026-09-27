import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    map_merge_dir = get_package_share_directory('multirobot_map_merge')
    launch_dir_map_merge = os.path.join(map_merge_dir, 'launch', 'tb3_simulation')
    tb3_sim_launch = os.path.join(launch_dir_map_merge, 'tb3_simulation_launch.py')
    map_merge_launch = os.path.join(map_merge_dir, 'launch', 'map_merge.launch.py')

    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    known_init_poses = LaunchConfiguration('known_init_poses')
    slam_toolbox = LaunchConfiguration('slam_toolbox')
    slam_gmapping = LaunchConfiguration('slam_gmapping')

    robot1_x = LaunchConfiguration('robot1_x')
    robot1_y = LaunchConfiguration('robot1_y')
    robot2_x = LaunchConfiguration('robot2_x')
    robot2_y = LaunchConfiguration('robot2_y')

    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value='/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world',
            description='Full path to world file to load'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
            description='Full path to map file to load'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('known_init_poses', default_value='False'),
        DeclareLaunchArgument('slam_toolbox', default_value='False'),
        DeclareLaunchArgument('slam_gmapping', default_value='True'),
        DeclareLaunchArgument('robot1_x', default_value='0.0'),
        DeclareLaunchArgument('robot1_y', default_value='0.5'),
        DeclareLaunchArgument('robot2_x', default_value='-3.0'),
        DeclareLaunchArgument('robot2_y', default_value='1.5'),
    ]

    # ---- 1. Gazebo ----
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen',
    )

    # ---- 2. Boundary fence (written to /tmp, spawned after Gazebo is up) ----
    fence_specs = [
        ('fence_1', 15.0, 0.0, 0.3, 30.0),
        ('fence_2', -15.0, 0.0, 0.3, 30.0),
        ('fence_3', 0.0, 15.0, 30.0, 0.3),
        ('fence_4', 0.0, -15.0, 30.0, 0.3),
    ]
    fence_actions = []
    for name, x, y, length, width in fence_specs:
        sdf_path = f'/tmp/{name}.sdf'
        with open(sdf_path, 'w') as f:
            f.write(
                f"<sdf version='1.6'><model name='{name}'><static>true</static>"
                f"<link name='link'><collision name='col'><geometry><box>"
                f"<size>{length} {width} 1.0</size></box></geometry></collision>"
                f"<visual name='vis'><geometry><box><size>{length} {width} 1.0</size>"
                f"</box></geometry></visual></link></model></sdf>"
            )
        fence_actions.append(ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', name, '-file', sdf_path,
                 '-x', str(x), '-y', str(y), '-z', '0.5'],
            output='screen',
        ))
    fence_cmd = TimerAction(period=8.0, actions=fence_actions)

    # ---- 3. per-robot spawn + SLAM + Nav2 ----
    def robot_group(name, x_pose, y_pose, params_arg_name):
        return GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(tb3_sim_launch),
                launch_arguments={
                    'namespace': name,
                    'use_namespace': 'True',
                    'map': map_yaml_file,
                    'use_sim_time': 'True',
                    'params_file': os.path.join(
                        launch_dir_map_merge, 'config', f'nav2_multirobot_{params_arg_name}.yaml'),
                    'autostart': autostart,
                    'use_rviz': 'False',
                    'use_simulator': 'False',
                    'headless': 'False',
                    'slam': 'True',
                    'slam_toolbox': slam_toolbox,
                    'slam_gmapping': slam_gmapping,
                    'use_robot_state_pub': 'True',
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'z_pose': '0.01',
                    'roll': '0.0',
                    'pitch': '0.0',
                    'yaw': '0.0',
                    'robot_name': name,
                }.items(),
            ),
        ])

    robot1_group = robot_group('robot1', robot1_x, robot1_y, 'params_1')
    robot2_group = robot_group('robot2', robot2_x, robot2_y, 'params_2')

    # ---- 4. map_merge ----
    map_merge_cmd = TimerAction(
        period=25.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_merge_launch),
            launch_arguments={
                'known_init_poses': known_init_poses,
                'use_sim_time': 'true',
            }.items(),
        )],
    )

    # ---- 5. explore_lite (fixed: costmap_topic = raw SLAM map, not nav2 costmap) ----
    def explore_node(namespace):
        return Node(
            package='explore_lite',
            name='explore_node',
            executable='explore',
            namespace=namespace,
            output='screen',
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            parameters=[{
                'use_sim_time': True,
                'robot_base_frame': 'base_link',
                'costmap_topic': 'map',
                'costmap_updates_topic': 'map_updates',
                'visualize': True,
                'planner_frequency': 0.33,
                'progress_timeout': 30.0,
                'potential_scale': 3.0,
                'orientation_scale': 0.0,
                'gain_scale': 1.0,
                'transform_tolerance': 0.3,
                'min_frontier_size': 0.5,
            }],
        )

    explore_cmd = TimerAction(
        period=30.0,
        actions=[explore_node('robot1'), explore_node('robot2')],
    )

    ld = LaunchDescription()
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(fence_cmd)
    ld.add_action(robot1_group)
    ld.add_action(robot2_group)
    ld.add_action(map_merge_cmd)
    ld.add_action(explore_cmd)
    return ld