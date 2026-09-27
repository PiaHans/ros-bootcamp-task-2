#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    import ast

    namespaces = ast.literal_eval(
        LaunchConfiguration('robot_namespaces').perform(context))
    poses = [float(v) for v in
             ast.literal_eval(LaunchConfiguration('init_poses').perform(context))]

    merger = Node(
        package='multi_bot_core', executable='map_merger',
        name='map_merger', output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_namespaces': namespaces,
                     'init_poses': poses,
                     'merged_topic': '/map',
                     'global_frame': 'map',
                     'publish_period': 1.0}],
    )

    explorer = Node(
        package='multi_bot_core', executable='explorer',
        name='explorer', output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
        parameters=[{'use_sim_time': True,
                     'robot_namespaces': namespaces,
                     'map_topic': '/map',
                     'global_frame': 'map',
                     'home_poses': poses}],
    )
    return [merger, explorer]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespaces', default_value="['tb3_0','tb3_1']"),
        DeclareLaunchArgument('init_poses',
                              default_value='[-4.0, -4.0, 0.0, 4.0, 4.0, 3.1416]'),
        DeclareLaunchArgument('explore', default_value='true'),
        OpaqueFunction(function=setup),
    ])
