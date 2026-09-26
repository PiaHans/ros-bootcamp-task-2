#!/usr/bin/env python3

import os
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_robot_sdf(original_sdf, prefix):
    """
    Generate a robot-specific SDF file with namespaced sensor and odometry frames.
    """
    tree = ET.parse(original_sdf)
    root = tree.getroot()

    for tag in root.iter('odometry_frame'):
        tag.text = f'{prefix}/odom'

    for tag in root.iter('robot_base_frame'):
        tag.text = f'{prefix}/base_footprint'

    for tag in root.iter('frame_name'):
        tag.text = f'{prefix}/base_scan'

    fd, output_path = tempfile.mkstemp(prefix=f'{prefix}_', suffix='.sdf')
    os.close(fd)

    tree.write(output_path, encoding='unicode', xml_declaration=True)
    return output_path


def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Allow fallback if turtlebot3_gazebo is available
    try:
        pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    except Exception:
        pkg_tb3_gazebo = None

    try:
        pkg_tb3_desc = get_package_share_directory('turtlebot3_description')
    except Exception:
        pkg_tb3_desc = None

    robot_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')

    # Default world
    default_world = os.path.join(pkg_simulation, 'worlds', 'maze.world')

    # Model paths
    if pkg_tb3_gazebo:
        original_sdf = os.path.join(
            pkg_tb3_gazebo, 'models', f'turtlebot3_{robot_model}', 'model.sdf'
        )
    else:
        original_sdf = os.path.join(
            '/opt/ros/humble/share/turtlebot3_gazebo/models',
            f'turtlebot3_{robot_model}',
            'model.sdf',
        )

    if pkg_tb3_desc:
        urdf_path = os.path.join(
            pkg_tb3_desc, 'urdf', f'turtlebot3_{robot_model}.urdf'
        )
    elif pkg_tb3_gazebo:
        urdf_path = os.path.join(
            pkg_tb3_gazebo, 'urdf', f'turtlebot3_{robot_model}.urdf'
        )
    else:
        urdf_path = f'/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_{robot_model}.urdf'

    # Launch Configurations
    world = LaunchConfiguration('world')
    robot1_x = LaunchConfiguration('robot1_x')
    robot1_y = LaunchConfiguration('robot1_y')
    robot1_yaw = LaunchConfiguration('robot1_yaw')
    robot2_x = LaunchConfiguration('robot2_x')
    robot2_y = LaunchConfiguration('robot2_y')
    robot2_yaw = LaunchConfiguration('robot2_yaw')

    # Generate isolated robot SDFs if source exists
    if os.path.exists(original_sdf):
        robot1_sdf = generate_robot_sdf(original_sdf, 'robot1')
        robot2_sdf = generate_robot_sdf(original_sdf, 'robot2')
    else:
        robot1_sdf = original_sdf
        robot2_sdf = original_sdf

    robot_desc = ''
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_desc = f.read()

    # 1. Gazebo Server & Client
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 2. Robot 1 Group
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': 'robot1/',
        }],
    )

    robot1_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot1',
        arguments=[
            '-entity', 'robot1',
            '-file', robot1_sdf,
            '-x', robot1_x,
            '-y', robot1_y,
            '-z', '0.01',
            '-Y', robot1_yaw,
            '-robot_namespace', '/robot1',
        ],
        output='screen',
    )

    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        robot1_state_pub,
        robot1_spawn,
    ])

    # 3. Robot 2 Group
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': 'robot2/',
        }],
    )

    robot2_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot2',
        arguments=[
            '-entity', 'robot2',
            '-file', robot2_sdf,
            '-x', robot2_x,
            '-y', robot2_y,
            '-z', '0.01',
            '-Y', robot2_yaw,
            '-robot_namespace', '/robot2',
        ],
        output='screen',
    )

    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        robot2_state_pub,
        robot2_spawn,
    ])

    # Cleanup temporary SDF files on shutdown
    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: [
                os.remove(p) for p in [robot1_sdf, robot2_sdf] if os.path.exists(p) and p.startswith(tempfile.gettempdir())
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world file'),
        DeclareLaunchArgument('robot1_x', default_value='-2.5', description='Robot 1 initial X position'),
        DeclareLaunchArgument('robot1_y', default_value='0.0', description='Robot 1 initial Y position'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0', description='Robot 1 initial Yaw'),
        DeclareLaunchArgument('robot2_x', default_value='2.5', description='Robot 2 initial X position'),
        DeclareLaunchArgument('robot2_y', default_value='0.0', description='Robot 2 initial Y position'),
        DeclareLaunchArgument('robot2_yaw', default_value='3.14159', description='Robot 2 initial Yaw'),

        gzserver,
        gzclient,
        cleanup,
        robot1_group,
        robot2_group,
    ])
