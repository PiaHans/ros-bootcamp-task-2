import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    launch_nav2 = LaunchConfiguration('launch_nav2').perform(context) == 'true'

    nodes = []

    if launch_nav2:
        bringup_dir = get_package_share_directory('nav2_bringup')
        nav_launch_dir = os.path.join(bringup_dir, 'launch')
        param_file = os.path.join(
            bringup_dir, 'params',
            'nav2_multirobot_params_1.yaml' if namespace == 'robot1' else 'nav2_multirobot_params_2.yaml'
        )

        nav2_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'true' if use_sim_time else 'false',
                'autostart': 'true',
                'params_file': param_file,
                'use_composition': 'False'
            }.items()
        )
        nodes.append(nav2_cmd)

    exp_node = Node(
        package='exploration',
        executable='frontier_exploration',
        name='frontier_exploration',
        namespace=namespace,
        parameters=[{
            'robot_namespace': namespace,
            'use_sim_time': use_sim_time,
            'min_frontier_size': 3
        }],
        output='screen'
    )
    nodes.append(exp_node)
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1', description='Target robot namespace for exploration'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('launch_nav2', default_value='false', description='Launch Nav2 stack alongside exploration'),
        OpaqueFunction(function=launch_setup)
    ])
