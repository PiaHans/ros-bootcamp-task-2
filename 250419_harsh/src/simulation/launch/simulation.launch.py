import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    # 1. Clean the ${namespace} placeholder from the URDF for robot_state_publisher
    urdf_path = os.path.join(pkg_tb3_desc, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf_path, 'r') as f:
        robot_urdf = f.read().replace('${namespace}', '')

    # 2. SDF for Gazebo spawn with namespaced frames
    sdf_path = os.path.join(pkg_tb3_gazebo, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    with open(sdf_path, 'r') as f:
        robot_sdf = f.read()

    # Robot 1 SDF
    robot1_sdf = robot_sdf.replace(
        '<frame_name>base_scan</frame_name>',
        '<frame_name>robot1/base_scan</frame_name>'
    ).replace(
        '<odometry_frame>odom</odometry_frame>',
        '<odometry_frame>robot1/odom</odometry_frame>'
    ).replace(
        '<robot_base_frame>base_footprint</robot_base_frame>',
        '<robot_base_frame>robot1/base_footprint</robot_base_frame>'
    )
    r1_sdf_file = '/tmp/robot1_waffle_pi.sdf'
    with open(r1_sdf_file, 'w') as f:
        f.write(robot1_sdf)

    # Robot 2 SDF
    robot2_sdf = robot_sdf.replace(
        '<frame_name>base_scan</frame_name>',
        '<frame_name>robot2/base_scan</frame_name>'
    ).replace(
        '<odometry_frame>odom</odometry_frame>',
        '<odometry_frame>robot2/odom</odometry_frame>'
    ).replace(
        '<robot_base_frame>base_footprint</robot_base_frame>',
        '<robot_base_frame>robot2/base_footprint</robot_base_frame>'
    )
    r2_sdf_file = '/tmp/robot2_waffle_pi.sdf'
    with open(r2_sdf_file, 'w') as f:
        f.write(robot2_sdf)

    # Coordinates
    r1_x = LaunchConfiguration('robot1_x', default='-1.5')
    r1_y = LaunchConfiguration('robot1_y', default='-0.5')
    r1_z = LaunchConfiguration('robot1_z', default='0.01')
    r1_yaw = LaunchConfiguration('robot1_yaw', default='0.0')

    r2_x = LaunchConfiguration('robot2_x', default='1.5')
    r2_y = LaunchConfiguration('robot2_y', default='0.5')
    r2_z = LaunchConfiguration('robot2_z', default='0.01')
    r2_yaw = LaunchConfiguration('robot2_yaw', default='3.14159')

    world_path = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot 1 State Publisher (publishes cleanly to global /tf and /tf_static)
    r1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_urdf,
            'frame_prefix': 'robot1/'
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )

    # Robot 1 Spawner
    r1_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', r1_sdf_file,
            '-robot_namespace', 'robot1',
            '-x', r1_x,
            '-y', r1_y,
            '-z', r1_z,
            '-Y', r1_yaw
        ],
        output='screen'
    )

    # Robot 2 State Publisher
    r2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_urdf,
            'frame_prefix': 'robot2/'
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )

    # Robot 2 Spawner
    r2_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot2',
            '-file', r2_sdf_file,
            '-robot_namespace', 'robot2',
            '-x', r2_x,
            '-y', r2_y,
            '-z', r2_z,
            '-Y', r2_yaw
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot1_x', default_value='-1.5'),
        DeclareLaunchArgument('robot1_y', default_value='-0.5'),
        DeclareLaunchArgument('robot1_z', default_value='0.01'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot2_x', default_value='1.5'),
        DeclareLaunchArgument('robot2_y', default_value='0.5'),
        DeclareLaunchArgument('robot2_z', default_value='0.01'),
        DeclareLaunchArgument('robot2_yaw', default_value='3.14159'),
        gzserver,
        gzclient,
        r1_state_pub,
        r1_spawner,
        r2_state_pub,
        r2_spawner
    ])