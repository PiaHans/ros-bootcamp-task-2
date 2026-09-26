import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    tb3_model_path = os.path.join(pkg_tb3_gazebo, 'models')
    system_model_path = '/usr/share/gazebo-11/models'
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    combined_model_path = f"{tb3_model_path}:{system_model_path}:{existing}"

    world_path = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', combined_model_path),
        gzserver_cmd,
        gzclient_cmd
    ])
