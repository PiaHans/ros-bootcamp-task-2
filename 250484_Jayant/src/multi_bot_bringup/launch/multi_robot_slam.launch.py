#!/usr/bin/env python3

import os
import tempfile

import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PKG = 'multi_bot_bringup'

def _materialise_params(template_path, namespace, nest_under_namespace):
   
    with open(template_path, 'r') as f:
        text = f.read().replace('<ns>', namespace)

    if nest_under_namespace:
        doc = yaml.safe_load(text)
        doc = {namespace: doc}
        text = yaml.dump(doc, default_flow_style=False)

    tmp_dir = os.path.join(tempfile.gettempdir(), 'multi_bot_params')
    os.makedirs(tmp_dir, exist_ok=True)
    out = os.path.join(tmp_dir, f'{namespace}_{os.path.basename(template_path)}')
    out = out.replace('.template', '')
    with open(out, 'w') as f:
        f.write(text)
    return out


def _nav2_nodes(namespace, params_file):
   
    lifecycle_nodes = ['controller_server', 'smoother_server', 'planner_server',
                       'behavior_server', 'bt_navigator', 'waypoint_follower',
                       'velocity_smoother']

    common = dict(namespace=namespace, output='screen', parameters=[params_file])

    return [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server',
             remappings=[('cmd_vel', 'cmd_vel_nav')], **common),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', **common),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **common),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')], **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=namespace,
             output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': True,
                          'bond_timeout': 0.0,
                          'node_names': lifecycle_nodes}]),
    ]


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory(PKG)

    xacro_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_waffle.urdf.xacro')
    slam_tpl = os.path.join(pkg_share, 'config', 'slam_params.template.yaml')
    nav2_tpl = LaunchConfiguration('nav2_params_file').perform(context)

    def f(name):
        return float(LaunchConfiguration(name).perform(context))

    def s(name):
        return LaunchConfiguration(name).perform(context)

    robots = [
        {'name': s('r1_name'), 'x': f('r1_x'), 'y': f('r1_y'), 'yaw': f('r1_yaw')},
        {'name': s('r2_name'), 'x': f('r2_x'), 'y': f('r2_y'), 'yaw': f('r2_yaw')},
    ]

    actions = []

    for i, rb in enumerate(robots):
        ns = rb['name']

        urdf_xml = xacro.process_file(
            xacro_file,
            mappings={'prefix': f'{ns}/', 'namespace': f'/{ns}',
                      'lidar_range': s('lidar_range')}
        ).toxml()

        slam_params = _materialise_params(slam_tpl, ns, nest_under_namespace=False)
        nav2_params = _materialise_params(nav2_tpl, ns, nest_under_namespace=True)

        rsp = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=ns, output='screen',
            parameters=[{'robot_description': urdf_xml,
                         'use_sim_time': True,
                         'frame_prefix': f'{ns}/'}],
        )

        map_link = Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'static_map_{ns}', output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'map', '--child-frame-id', f'{ns}/map'],
        )

        spawn = Node(
            package='gazebo_ros', executable='spawn_entity.py',
            name=f'spawn_{ns}', output='screen',
            arguments=['-entity', ns,
                       '-topic', f'/{ns}/robot_description',
                       '-x', str(rb['x']), '-y', str(rb['y']), '-z', '0.01',
                       '-Y', str(rb['yaw'])],
        )

        slam = Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace=ns, output='screen',
            parameters=[slam_params, {'use_sim_time': True}],
            remappings=[
                ('/map', f'/{ns}/map'),
                ('map', f'/{ns}/map'),
                ('/map_metadata', f'/{ns}/map_metadata'),
                ('map_metadata', f'/{ns}/map_metadata'),
            ],
        )

        after_spawn = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[TimerAction(period=2.0,
                                     actions=[slam] + _nav2_nodes(ns, nav2_params))],
            )
        )

        actions += [rsp, map_link,
                    TimerAction(period=4.0 + 2.0 * i, actions=[spawn]),
                    after_spawn]

    ns_list = [rb['name'] for rb in robots]
    init_poses = []
    for rb in robots:
        init_poses += [rb['x'], rb['y'], rb['yaw']]

    merger = Node(
        package='multi_bot_core', executable='map_merger',
        name='map_merger', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': ns_list,
            'merged_topic': '/map',
            'global_frame': 'map',
            'publish_period': 1.0,
        }],
    )

    explorer = Node(
        package='multi_bot_core', executable='explorer',
        name='explorer', output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': ns_list,
            'map_topic': '/map',
            'global_frame': 'map',
            'base_frame_suffix': 'base_footprint',
            'planning_period': 2.0,
            'min_frontier_cells': 10,
            'robot_radius': 0.25,
            'goal_clearance': 0.30,
            'progress_timeout': 40.0,
            'blacklist_radius': 0.60,
            'gain_radius': 1.5,
            'w_distance': 1.0,
            'w_gain': 0.35,
            'separation_bonus': 2.0,
            'finish_after_empty_cycles': 5,
            'return_home': True,
            'home_poses': init_poses,
        }],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    actions += [
        TimerAction(period=10.0, actions=[merger]),
        TimerAction(period=25.0, actions=[explorer]),
        TimerAction(period=8.0, actions=[rviz]),
    ]

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    default_world = os.path.join(pkg_share, 'worlds', 'enclosed_world.world')
    default_nav2 = os.path.join(pkg_share, 'config', 'nav2_params.template.yaml')

    declare = [
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2),
        DeclareLaunchArgument('gui', default_value='true',
                              description='run gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('explore', default_value='true',
                              description='start autonomous frontier exploration'),
        DeclareLaunchArgument('lidar_range', default_value='6.0'),

        DeclareLaunchArgument('r1_name', default_value='tb3_0'),
        DeclareLaunchArgument('r1_x', default_value='-4.0'),
        DeclareLaunchArgument('r1_y', default_value='-4.0'),
        DeclareLaunchArgument('r1_yaw', default_value='0.0'),

        DeclareLaunchArgument('r2_name', default_value='tb3_1'),
        DeclareLaunchArgument('r2_x', default_value='4.0'),
        DeclareLaunchArgument('r2_y', default_value='4.0'),
        DeclareLaunchArgument('r2_yaw', default_value='3.1416'),
    ]

    gazebo_ros = get_package_share_directory('gazebo_ros')

    ros_share = os.path.dirname(get_package_share_directory('turtlebot3_description'))
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.environ.get('GAZEBO_MODEL_PATH', '') + os.pathsep + ros_share)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'verbose': 'false'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription(
        declare + [set_model_path, gzserver, gzclient,
                   OpaqueFunction(function=launch_setup)]
    )
