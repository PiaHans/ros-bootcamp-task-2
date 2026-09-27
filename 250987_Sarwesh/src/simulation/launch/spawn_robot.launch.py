import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # Load URDF for robot_state_publisher
    urdf_path = os.path.join(tb3_gazebo_share, 'urdf', f'turtlebot3_{tb3_model}.urdf')
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    # Load and adjust SDF for Gazebo simulation plugins
    sdf_path = os.path.join(tb3_gazebo_share, 'models', f'turtlebot3_{tb3_model}', 'model.sdf')
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for odom_tag in root.iter('odometry_frame'):
        odom_tag.text = f'{robot_name}/odom'
    for base_tag in root.iter('robot_base_frame'):
        base_tag.text = f'{robot_name}/base_footprint'
    for scan_tag in root.iter('frame_name'):
        scan_tag.text = f'{robot_name}/base_scan'

    # Save customized SDF for this robot
    tmp_sdf_path = f'/tmp/{robot_name}_{tb3_model}.sdf'
    sdf_string = '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding='unicode')
    with open(tmp_sdf_path, 'w') as f:
        f.write(sdf_string)

    # Robot State Publisher: publishes 3D transforms for robot1
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': urdf_content,
            'frame_prefix': f'{robot_name}/'
        }]
    )

    # Spawner: connects to global /spawn_entity service to instantiate robot
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{robot_name}',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', tmp_sdf_path,
            '-robot_namespace', robot_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ]
    )

    return [rsp_node, spawn_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1', description='Robot namespace and name'),
        DeclareLaunchArgument('x_pose', default_value='-3.0', description='Initial X position'),
        DeclareLaunchArgument('y_pose', default_value='-3.0', description='Initial Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.01', description='Initial Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw angle'),
        OpaqueFunction(function=launch_setup)
    ])
