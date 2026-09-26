#!/usr/bin/env python3
import math
import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value

        self.map_topic = f'/{self.robot_name}/map'
        self.nav_action = f'/{self.robot_name}/navigate_to_pose'
        self.global_frame = f'{self.robot_name}/map'
        self.base_frame = f'{self.robot_name}/base_footprint'

        self.get_logger().info(f'Starting Frontier Explorer for {self.robot_name}')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_map = None
        self.is_navigating = False
        self.goal_handle = None
        self.failed_goals = []  # List of (x, y, timestamp)

        self.timer = self.create_timer(2.0, self.exploration_loop)

    def map_callback(self, msg):
        self.current_map = msg

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return (trans.transform.translation.x, trans.transform.translation.y)
        except Exception:
            return None

    def find_frontiers(self):
        if self.current_map is None:
            return []

        # Prune failed goals older than 20 seconds
        now = time.time()
        self.failed_goals = [(fx, fy, t) for fx, fy, t in self.failed_goals if now - t < 20.0]

        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        grid = np.array(self.current_map.data, dtype=np.int8).reshape((height, width))

        free_mask = (grid == 0).astype(np.uint8)
        unknown_mask = (grid == -1).astype(np.uint8)
        obstacle_mask = (grid > 50).astype(np.uint8)

        kernel = np.ones((3, 3), np.uint8)
        inflated_obstacles = cv2.dilate(obstacle_mask, kernel, iterations=3)
        safe_free = free_mask & (~inflated_obstacles)
        dilated_unknown = cv2.dilate(unknown_mask, kernel, iterations=1)
        frontier_mask = safe_free & dilated_unknown

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask)
        if num_labels <= 1:
            return []

        frontiers = []
        robot_pose = self.get_robot_pose()

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 5:  # Filter out noise
                continue

            cx, cy = centroids[i]
            wx = origin_x + (cx + 0.5) * resolution
            wy = origin_y + (cy + 0.5) * resolution

            too_close_to_failed = any(
                math.hypot(wx - fx, wy - fy) < 0.4 for fx, fy, _ in self.failed_goals
            )
            if too_close_to_failed:
                continue

            dist = 0.0
            if robot_pose is not None:
                dist = math.hypot(wx - robot_pose[0], wy - robot_pose[1])

            frontiers.append({'x': wx, 'y': wy, 'dist': dist, 'size': area})

        frontiers.sort(key=lambda item: item['dist'])
        return frontiers

    def exploration_loop(self):
        if self.is_navigating or self.current_map is None:
            return

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info(f'[{self.robot_name}] No reachable frontiers found.')
            return

        target = frontiers[0]
        self.get_logger().info(f'[{self.robot_name}] Driving to frontier at ({target["x"]:.2f}, {target["y"]:.2f}) [dist: {target["dist"]:.2f}m]')
        self.send_goal(target['x'], target['y'])

    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return

        robot_pose = self.get_robot_pose()
        yaw = 0.0
        if robot_pose is not None:
            yaw = math.atan2(y - robot_pose[1], x - robot_pose[0])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.is_navigating = True
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda future: self.goal_response_callback(future, (x, y)))

    def goal_response_callback(self, future, target):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn(f'[{self.robot_name}] Goal rejected at ({target[0]:.2f}, {target[1]:.2f})')
            self.failed_goals.append((target[0], target[1], time.time()))
            self.is_navigating = False
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_callback(f, target))

    def result_callback(self, future, target):
        self.is_navigating = False
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'[{self.robot_name}] Reached frontier at ({target[0]:.2f}, {target[1]:.2f})!')
        else:
            self.failed_goals.append((target[0], target[1], time.time()))

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
