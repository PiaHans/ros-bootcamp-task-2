import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    urdf_path = os.path.join(pkg_tb3_gazebo, 'urdf', f'turtlebot3_{robot_model}.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read().replace('${namespace}', '')

    model_sdf_path = os.path.join(
        pkg_tb3_gazebo, 'models', f'turtlebot3_{robot_model}', 'model.sdf'
    )

    import xml.etree.ElementTree as ET

    tree = ET.parse(model_sdf_path)
    root = tree.getroot()
    if robot_name:
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{robot_name}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{robot_name}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{robot_name}/base_scan'

    tmp_sdf_path = f'/tmp/{robot_name}_model.sdf'
    tree.write(tmp_sdf_path)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': f'{robot_name}/'
        }],
        output='screen'
    )

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', tmp_sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
            '-robot_namespace', robot_name
        ],
        output='screen'
    )

    return [rsp_node, spawn_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1', description='Robot namespace and entity name'),
        DeclareLaunchArgument('robot_model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'waffle'), description='TurtleBot3 model'),
        DeclareLaunchArgument('x_pose', default_value='-2.0', description='Initial X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.01', description='Initial Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw orientation'),
        OpaqueFunction(function=launch_setup)
    ])
