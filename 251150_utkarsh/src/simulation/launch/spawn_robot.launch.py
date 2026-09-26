import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def spawn_robot_function(context, *args, **kwargs):
    robot_name = context.launch_configurations['robot_name']
    x_pose = context.launch_configurations['x_pose']
    y_pose = context.launch_configurations['y_pose']
    yaw_pose = context.launch_configurations['yaw_pose']

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    # Load and adjust SDF model for namespaced frames
    sdf_path = os.path.join(pkg_tb3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf')
    with open(sdf_path, 'r') as f:
        sdf_content = f.read()

    # Prefix frame names inside Gazebo plugins so TF frames don't collide
    sdf_content = sdf_content.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{robot_name}/base_scan</frame_name>')
    sdf_content = sdf_content.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{robot_name}/odom</odometry_frame>')
    sdf_content = sdf_content.replace('<robot_base_frame>base_footprint</robot_base_frame>', f'<robot_base_frame>{robot_name}/base_footprint</robot_base_frame>')

    tmp_sdf = os.path.join(tempfile.gettempdir(), f'{robot_name}.sdf')
    with open(tmp_sdf, 'w') as f:
        f.write(sdf_content)

    # Robot State Publisher for URDF TF tree
    urdf_path = os.path.join(pkg_tb3_gazebo, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
            'frame_prefix': f'{robot_name}/'
        }]
    )

    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', tmp_sdf,
            '-robot_namespace', robot_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.05',
            '-Y', yaw_pose
        ],
        output='screen'
    )

    return [node_robot_state_publisher, node_spawn_entity]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw_pose', default_value='0.0'),
        OpaqueFunction(function=spawn_robot_function)
    ])
