#!/usr/bin/env python3
"""
Autonomous Frontier Exploration Node for ROS 2 Humble.
Implements Yamauchi frontier-based exploration with connected-component
clustering, obstacle clearance checks, goal blacklisting, and Nav2 action client.
"""

import math
import numpy as np
from scipy.ndimage import binary_dilation, label, distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener


class FrontierExplorer(Node):
    """Yamauchi Frontier-Based Autonomous Exploration Node."""

    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('map_frame', '')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('map_topic', '')
        self.declare_parameter('min_frontier_size', 4)
        self.declare_parameter('min_goal_distance', 0.4)
        self.declare_parameter('min_obstacle_clearance', 0.28)
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('blacklist_radius', 0.35)
        self.declare_parameter('goal_timeout_sec', 60.0)
        self.declare_parameter('rate', 1.0)

        # Resolve namespace and frames
        ns_param = self.get_parameter('robot_namespace').get_parameter_value().string_value
        node_ns = self.get_namespace().strip('/')
        self.robot_namespace = ns_param or node_ns or 'robot1'

        map_frame_param = self.get_parameter('map_frame').get_parameter_value().string_value
        base_frame_param = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = map_frame_param or f'{self.robot_namespace}/map'
        self.base_frame = base_frame_param or f'{self.robot_namespace}/base_footprint'

        map_topic_param = self.get_parameter('map_topic').get_parameter_value().string_value
        self.map_topic = map_topic_param or f'/{self.robot_namespace}/map'

        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value
        self.min_goal_distance = self.get_parameter('min_goal_distance').get_parameter_value().double_value
        self.min_obstacle_clearance = self.get_parameter('min_obstacle_clearance').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().integer_value
        self.blacklist_radius = self.get_parameter('blacklist_radius').get_parameter_value().double_value
        self.goal_timeout_sec = self.get_parameter('goal_timeout_sec').get_parameter_value().double_value
        rate = self.get_parameter('rate').get_parameter_value().double_value

        self.get_logger().info(
            f'FrontierExplorer initialized: ns={self.robot_namespace}, '
            f'map_topic={self.map_topic}, map_frame={self.map_frame}, base_frame={self.base_frame}, '
            f'min_clearance={self.min_obstacle_clearance}m'
        )

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map subscription with Transient Local QoS (matching SLAM Toolbox)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)

        # Marker publisher for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'frontier_markers', 10)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State tracking
        self.latest_map = None
        self.blacklist = []
        self.moving = False
        self.current_goal = None
        self.goal_handle = None
        self.goal_start_time = None
        self.no_frontiers_count = 0
        self.exploration_complete = False

        # Periodic exploration timer
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.exploration_step)

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def get_robot_pose(self):
        """Look up the robot current position in the map frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def detect_frontiers(self, map_msg: OccupancyGrid):
        """Detect and cluster Yamauchi frontier cells."""
        width = map_msg.info.width
        height = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        if width < 5 or height < 5:
            return []

        grid = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # Masks
        free_mask = (grid == 0)
        unknown_mask = (grid == -1)
        obstacle_mask = (grid >= self.obstacle_threshold)

        # Dilate unknown cells to find adjacent free cells
        unknown_dilated = binary_dilation(unknown_mask, structure=np.ones((3, 3)))

        # Distance transform: distance from every cell to nearest obstacle in meters
        dist_to_obs = distance_transform_edt(~obstacle_mask) * res
        safe_clearance = (dist_to_obs >= self.min_obstacle_clearance)

        # Frontier cells: free, adjacent to unknown, and safe clearance from obstacles
        frontier_mask = free_mask & unknown_dilated & safe_clearance

        # Group contiguous frontier cells into clusters
        labeled, num_features = label(frontier_mask, structure=np.ones((3, 3)))

        candidates = []
        for i in range(1, num_features + 1):
            pts = np.argwhere(labeled == i)
            if len(pts) < self.min_frontier_size:
                continue

            # Centroid in grid coordinates
            cy, cx = np.mean(pts, axis=0)

            # Representative cell: point in cluster closest to centroid
            best_idx = np.argmin((pts[:, 0] - cy) ** 2 + (pts[:, 1] - cx) ** 2)
            by, bx = pts[best_idx]

            # Convert to world coordinates
            wx = ox + (bx + 0.5) * res
            wy = oy + (by + 0.5) * res

            # Check if this point is in the blacklist
            if any(math.hypot(wx - bx_bl, wy - by_bl) < self.blacklist_radius for bx_bl, by_bl in self.blacklist):
                continue

            candidates.append((wx, wy, len(pts)))

        return candidates

    def publish_markers(self, candidates, active_goal=None):
        """Publish markers to RViz for debugging and visualization."""
        marker_array = MarkerArray()

        # Points marker for all valid frontiers
        frontier_marker = Marker()
        frontier_marker.header.frame_id = self.map_frame
        frontier_marker.header.stamp = self.get_clock().now().to_msg()
        frontier_marker.ns = 'frontiers'
        frontier_marker.id = 0
        frontier_marker.type = Marker.POINTS
        frontier_marker.action = Marker.ADD
        frontier_marker.scale.x = 0.08
        frontier_marker.scale.y = 0.08
        frontier_marker.color.r = 0.0
        frontier_marker.color.g = 0.8
        frontier_marker.color.b = 1.0
        frontier_marker.color.a = 1.0

        for wx, wy, _ in candidates:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.05
            frontier_marker.points.append(p)

        marker_array.markers.append(frontier_marker)

        # Sphere marker for the currently targeted goal
        goal_marker = Marker()
        goal_marker.header.frame_id = self.map_frame
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'active_goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2

        if active_goal is not None:
            goal_marker.pose.position.x = active_goal[0]
            goal_marker.pose.position.y = active_goal[1]
            goal_marker.pose.position.z = 0.1
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.1
            goal_marker.color.b = 0.1
            goal_marker.color.a = 1.0
        else:
            goal_marker.action = Marker.DELETE

        marker_array.markers.append(goal_marker)
        self.marker_pub.publish(marker_array)

    def exploration_step(self):
        """Main exploration control cycle."""
        if self.exploration_complete:
            return

        # Check navigation state and timeout
        if self.moving:
            if self.goal_start_time is not None:
                elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout_sec:
                    self.get_logger().warn(f'Navigation timed out ({elapsed:.1f}s). Cancelling and blacklisting.')
                    if self.goal_handle:
                        self.goal_handle.cancel_goal_async()
                    if self.current_goal:
                        self.blacklist.append(self.current_goal)
                    self.reset_nav_state()
            return

        if self.latest_map is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return

        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().info('Waiting for TF (robot pose)...', throttle_duration_sec=5.0)
            return

        rx, ry = pose
        candidates = self.detect_frontiers(self.latest_map)

        if not candidates:
            self.no_frontiers_count += 1
            if self.no_frontiers_count >= 5:
                self.get_logger().info('*** Exploration Complete: No reachable frontiers remain. ***')
                self.exploration_complete = True
                self.publish_markers([], active_goal=None)
            return

        self.no_frontiers_count = 0

        # Filter out frontiers too close to current position to avoid local oscillation
        valid = [c for c in candidates if math.hypot(c[0] - rx, c[1] - ry) >= self.min_goal_distance]
        if not valid:
            valid = candidates

        # Yamauchi policy: choose the closest valid frontier
        valid.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))
        target_x, target_y, _ = valid[0]

        self.publish_markers(candidates, active_goal=(target_x, target_y))
        self.send_navigation_goal(target_x, target_y, rx, ry)

    def send_navigation_goal(self, gx, gy, rx, ry):
        """Send NavigateToPose action goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 navigate_to_pose action server not ready yet.')
            return

        yaw = math.atan2(gy - ry, gx - rx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.moving = True
        self.current_goal = (gx, gy)
        self.goal_start_time = self.get_clock().now()

        dist = math.hypot(gx - rx, gy - ry)
        self.get_logger().info(f'Dispatching goal to ({gx:.2f}, {gy:.2f}), dist={dist:.2f}m')

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2. Blacklisting.')
            if self.current_goal:
                self.blacklist.append(self.current_goal)
            self.reset_nav_state()
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Frontier goal reached successfully.')
        else:
            self.get_logger().warn(f'Navigation failed (status {status}). Blacklisting goal.')
            if self.current_goal:
                self.blacklist.append(self.current_goal)

        self.reset_nav_state()

    def reset_nav_state(self):
        self.moving = False
        self.current_goal = None
        self.goal_handle = None
        self.goal_start_time = None


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
