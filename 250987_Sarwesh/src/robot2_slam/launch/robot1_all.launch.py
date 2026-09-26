import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_simulation = get_package_share_directory('simulation')
    pkg_robot2_slam = get_package_share_directory('robot2_slam')

    # 1. Gazebo + Robot Spawner (Starts at t=0s)
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'simulation.launch.py')
        )
    )

    # 2. SLAM Toolbox (Starts at t=5s after robot spawns)
    slam_cmd = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot2_slam, 'launch', 'slam.launch.py')
                )
            )
        ]
    )

    # 3. Nav2 Navigation Stack (Starts at t=10s after SLAM initializes)
    nav2_cmd = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot2_slam, 'launch', 'nav2.launch.py')
                )
            )
        ]
    )

    # 4. Pre-configured RViz (Starts at t=12s)
    rviz_cmd = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot2_slam, 'launch', 'rviz.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        sim_cmd,
        slam_cmd,
        nav2_cmd,
        rviz_cmd
    ])
