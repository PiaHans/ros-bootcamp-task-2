#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')
    pkg_robot1_slam = get_package_share_directory('robot1_slam')
    pkg_robot2_slam = get_package_share_directory('robot2_slam')
    pkg_bringup = get_package_share_directory('bringup')

    # Default world
    default_world = os.path.join(pkg_simulation, 'worlds', 'maze.world')
    default_rviz = os.path.join(pkg_bringup, 'rviz', 'multi_robot.rviz')

    # Launch Configurations
    world = LaunchConfiguration('world')
    robot1_x = LaunchConfiguration('robot1_x')
    robot1_y = LaunchConfiguration('robot1_y')
    robot1_yaw = LaunchConfiguration('robot1_yaw')
    robot2_x = LaunchConfiguration('robot2_x')
    robot2_y = LaunchConfiguration('robot2_y')
    robot2_yaw = LaunchConfiguration('robot2_yaw')
    use_rviz = LaunchConfiguration('use_rviz')

    # 1. Simulation and Robot Spawning
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'gazebo_and_spawn.launch.py')
        ),
        launch_arguments={
            'world': world,
            'robot1_x': robot1_x,
            'robot1_y': robot1_y,
            'robot1_yaw': robot1_yaw,
            'robot2_x': robot2_x,
            'robot2_y': robot2_y,
            'robot2_yaw': robot2_yaw,
        }.items(),
    )

    # 2. SLAM for Robot 1 and Robot 2
    robot1_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot1_slam, 'launch', 'slam.launch.py')
        )
    )

    robot2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot2_slam, 'launch', 'slam.launch.py')
        )
    )

    # 3. Nav2 Navigation for Robot 1 and Robot 2
    robot1_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot1_slam, 'launch', 'robot1_nav2.launch.py')
        )
    )

    robot2_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot2_slam, 'launch', 'robot2_nav2.launch.py')
        )
    )

    # 4. Autonomous Frontier Explorers for Robot 1 and Robot 2
    robot1_explorer_node = Node(
        package='exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace='robot1',
        output='screen',
        parameters=[{
            'min_frontier_size': 5,
            'min_goal_distance': 0.6,
            'max_goal_distance': 15.0,
            'safety_distance': 0.35,
            'plan_interval': 3.0,
        }],
    )

    robot2_explorer_node = Node(
        package='exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace='robot2',
        output='screen',
        parameters=[{
            'min_frontier_size': 5,
            'min_goal_distance': 0.6,
            'max_goal_distance': 15.0,
            'safety_distance': 0.35,
            'plan_interval': 3.0,
        }],
    )

    # 5. Map Merger Node
    map_merger_node = Node(
        package='map_merger',
        executable='map_merger_node',
        name='map_merger_node',
        output='screen',
        parameters=[{
            'robot1_x': robot1_x,
            'robot1_y': robot1_y,
            'robot1_yaw': robot1_yaw,
            'robot2_x': robot2_x,
            'robot2_y': robot2_y,
            'robot2_yaw': robot2_yaw,
            'merge_frequency': 1.0,
            'feature_matching_min_inliers': 8,
        }],
    )

    # 6. RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world file'),
        DeclareLaunchArgument('robot1_x', default_value='-2.5', description='Robot 1 initial X position'),
        DeclareLaunchArgument('robot1_y', default_value='0.0', description='Robot 1 initial Y position'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0', description='Robot 1 initial Yaw'),
        DeclareLaunchArgument('robot2_x', default_value='2.5', description='Robot 2 initial X position'),
        DeclareLaunchArgument('robot2_y', default_value='0.0', description='Robot 2 initial Y position'),
        DeclareLaunchArgument('robot2_yaw', default_value='3.14159', description='Robot 2 initial Yaw'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RViz2'),

        # Execution nodes
        simulation_launch,
        robot1_slam_launch,
        robot2_slam_launch,
        robot1_nav2_launch,
        robot2_nav2_launch,
        robot1_explorer_node,
        robot2_explorer_node,
        map_merger_node,
        rviz_node,
    ])
