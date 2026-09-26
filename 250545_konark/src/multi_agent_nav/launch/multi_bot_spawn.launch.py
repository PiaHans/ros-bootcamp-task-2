import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_agent_nav')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    default_nav2_yaml = os.path.join(pkg_dir, 'config', 'nav2_multirobot.yaml')
    slam_yaml = os.path.join(pkg_dir, 'config', 'slam_multirobot.yaml')
    default_world = os.path.join(pkg_dir, 'worlds', 'maze.world')
    rviz_config = os.path.join(pkg_dir, 'config', 'multi_nav.rviz')

    # Launch Configurations
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_explore = LaunchConfiguration('use_explore')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    r1_x = LaunchConfiguration('r1_x')
    r1_y = LaunchConfiguration('r1_y')
    r1_z = LaunchConfiguration('r1_z')
    r2_x = LaunchConfiguration('r2_x')
    r2_y = LaunchConfiguration('r2_y')
    r2_z = LaunchConfiguration('r2_z')

    # Declarations
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=default_world, description='World file path')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz2')
    declare_use_explore_cmd = DeclareLaunchArgument(
        'use_explore', default_value='true', description='Launch explore_lite')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_nav2_yaml,
        description='Path to Nav2 params YAML')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level')

    declare_r1_x = DeclareLaunchArgument(
        'r1_x', default_value='-2.0', description='X pos of bot 1')
    declare_r1_y = DeclareLaunchArgument('r1_y', default_value='0.0', description='Y pos of bot 1')
    declare_r1_z = DeclareLaunchArgument('r1_z', default_value='0.05', description='Z pos of bot 1')

    declare_r2_x = DeclareLaunchArgument('r2_x', default_value='2.0', description='X pos of bot 2')
    declare_r2_y = DeclareLaunchArgument('r2_y', default_value='0.0', description='Y pos of bot 2')
    declare_r2_z = DeclareLaunchArgument('r2_z', default_value='0.05', description='Z pos of bot 2')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    remappings = [
        ('cmd_vel', 'cmd_vel_nav'),
        ('cmd_vel_smoothed', 'cmd_vel')
    ]

    # All robots must publish their uniquely prefixed frames on the same TF
    # graph. A relative `tf` topic inside PushRosNamespace would otherwise
    # become /robotN/tf, which RViz and the other robot stacks do not read.
    tf_remappings = [
        ('tf', '/tf'),
        ('tf_static', '/tf_static')
    ]

    robots = [
        {'name': 'robot1', 'x': r1_x, 'y': r1_y, 'z': r1_z},
        {'name': 'robot2', 'x': r2_x, 'y': r2_y, 'z': r2_z}
    ]

    ld = LaunchDescription()

    # Add environment variables and declarations
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_explore_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_r1_x)
    ld.add_action(declare_r1_y)
    ld.add_action(declare_r1_z)
    ld.add_action(declare_r2_x)
    ld.add_action(declare_r2_y)
    ld.add_action(declare_r2_z)

    # 1. Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    ld.add_action(gazebo)

    # 2. Spawn and configure each robot
    for i, robot in enumerate(robots):
        namespace = robot['name']

        robot_desc_value = ParameterValue(
            Command(['xacro ', urdf_file, ' robot_name:=', namespace]),
            value_type=str
        )

        # State publisher publishes URDF frames to /tf
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc_value,
                'use_sim_time': use_sim_time
            }],
            remappings=tf_remappings
        )

        # Spawner places robot in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=[
                '-entity', namespace,
                '-topic', 'robot_description',
                '-x', robot['x'], '-y', robot['y'], '-z', robot['z']
            ],
            output='screen'
        )

        # SLAM Toolbox generates /<namespace>/map and publishes <namespace>/map -> <namespace>/odom
        slam_params = ParameterFile(
            RewrittenYaml(
                source_file=slam_yaml,
                root_key=namespace,
                param_rewrites={
                    'use_sim_time': use_sim_time,
                    'odom_frame': f'{namespace}/odom',
                    'base_frame': f'{namespace}/base_footprint',
                    'map_frame': f'{namespace}/map',
                    'scan_topic': f'/{namespace}/scan',
                    'map_name': f'/{namespace}/map'
                },
                convert_types=True
            ),
            allow_substs=True
        )

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {
                    'map_start_pose': [robot['x'], robot['y'], 0.0],
                    'map_start_at_dock': True
                }
            ],
            remappings=tf_remappings
        )

        # Frontier exploration node (explore_lite) sends navigate_to_pose goals to Nav2
        explore_node = Node(
            condition=IfCondition(use_explore),
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[{
                'robot_base_frame': f'{namespace}/base_footprint',
                'costmap_topic': f'/{namespace}/global_costmap/costmap',
                'costmap_updates_topic': f'/{namespace}/global_costmap/costmap_updates',
                'visualize': True,
                'planner_frequency': 0.33,
                'progress_timeout': 30.0,
                'potential_scale': 3.0,
                'orientation_scale': 0.0,
                'gain_scale': 1.0,
                'transform_tolerance': 0.3,
                'use_sim_time': use_sim_time
            }]
        )

        # Nav2 Navigation stack configuration for this robot
        param_substitutions = {
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }

        params_file_replaced = ReplaceString(
            source_file=params_file,
            replacements={'<robot_namespace>': namespace}
        )

        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file_replaced,
                root_key=namespace,
                param_rewrites=param_substitutions,
                convert_types=True
            ),
            allow_substs=True
        )

        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        )

        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]
        )

        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]
        )

        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]
        )

        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]
        )

        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]
        )

        velocity_smoother = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        )

        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 60.0,
                'bond_respawn_max_duration': 60.0
            }]
        )

        # PushRosNamespace groups all base and navigation nodes under the robot's namespace
        robot_group = GroupAction([
            PushRosNamespace(namespace),
            robot_state_publisher,
            spawn_entity,
            slam_node,
            explore_node,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager
        ])
        
        # Stagger the launch of each robot to prevent DDS discovery overload
        delay_time = i * 15.0
        delayed_robot_group = TimerAction(
            period=delay_time,
            actions=[robot_group]
        )
        ld.add_action(delayed_robot_group)

        # Static transform connecting global map to this robot's initial SLAM map frame
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_pub_{namespace}',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', f'{namespace}/map'
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )
        ld.add_action(static_tf)

    # 3. Map merge node to merge individual robot maps onto /map
    map_merge = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        parameters=[{
            'robot_map_topic': 'map',
            'robot_namespace': '',
            'merged_map_topic': '/map',
            'world_frame': 'map',
            'known_init_poses': False,
            'merging_rate': 1.0,
            'discovery_rate': 0.1,
            'estimation_rate': 0.5,
            'estimation_confidence': 1.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    ld.add_action(map_merge)

    # 4. RViz2 visualization (delayed 10.0s to allow Gazebo to spawn entities and publish TF)
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_timer = TimerAction(
        period=14.0,
        actions=[rviz_node]
    )
    ld.add_action(rviz_timer)

    return ld
