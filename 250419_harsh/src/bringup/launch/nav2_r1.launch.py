import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_bringup, 'config', 'nav2_r1.yaml')

    # TF Relay process to bridge global /tf to namespaced /robot1/tf
    tf_relay_cmd = ExecuteProcess(
        cmd=[
            'python3', '-c',
            'import rclpy; '
            'from rclpy.node import Node; '
            'from tf2_msgs.msg import TFMessage; '
            'from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy; '
            'rclpy.init(); '
            'node = Node("tf_relay_r1"); '
            'tf_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100); '
            'static_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=100); '
            'p1 = node.create_publisher(TFMessage, "/robot1/tf", tf_qos); '
            'p1_s = node.create_publisher(TFMessage, "/robot1/tf_static", static_qos); '
            'p2 = node.create_publisher(TFMessage, "/robot2/tf", tf_qos); '
            'p2_s = node.create_publisher(TFMessage, "/robot2/tf_static", static_qos); '
            'node.create_subscription(TFMessage, "/tf", lambda m: (p1.publish(m), p2.publish(m)), tf_qos); '
            'node.create_subscription(TFMessage, "/tf_static", lambda m: (p1_s.publish(m), p2_s.publish(m)), static_qos); '
            'rclpy.spin(node)'
        ],
        output='screen'
    )

    nav2_cmd = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace': 'robot1',
                'params_file': params_file,
                'autostart': 'true',
                'use_lifecycle_mgr': 'true',
                'map_subscribe_transient_local': 'true'
            }.items()
        )
    ])

    return LaunchDescription([
        tf_relay_cmd,
        nav2_cmd
    ])