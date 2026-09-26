#!/usr/bin/env python3

import copy
import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros


class FrontierExplorer(Node):
    """
    Autonomous Frontier-Based Explorer for Multi-Robot SLAM.
    Detects boundaries between explored free space and unknown territory,
    clusters frontiers, applies collision-safe filtering and regional bias,
    and commands Nav2 action server autonomously until full coverage.
    """

    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('min_goal_distance', 0.6)
        self.declare_parameter('max_goal_distance', 15.0)
        self.declare_parameter('safety_distance', 0.35)
        self.declare_parameter('plan_interval', 3.0)
        self.declare_parameter('max_goal_duration', 45.0)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.plan_interval = self.get_parameter('plan_interval').value
        self.max_goal_duration = self.get_parameter('max_goal_duration').value

        # Namespace and frame setup
        ns = self.get_namespace().strip('/')
        self.ns = ns if ns else ''
        self.map_frame = f'{self.ns}/map' if self.ns else 'map'
        self.base_frame = f'{self.ns}/base_footprint' if self.ns else 'base_footprint'

        # Bias configuration for multi-robot regional divergence
        # Robot 1 favors +y / left region; Robot 2 favors -y / right region
        if '1' in self.ns:
            self.bias_y_sign = 1.0
        elif '2' in self.ns:
            self.bias_y_sign = -1.0
        else:
            self.bias_y_sign = 0.0

        self.get_logger().info(
            f'Frontier Explorer initialized for namespace: "{self.ns}", '
            f'map_frame: "{self.map_frame}", base_frame: "{self.base_frame}", '
            f'bias_y_sign: {self.bias_y_sign}'
        )

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        action_name = 'navigate_to_pose'
        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        # Subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Internal State
        self.latest_map = None
        self.latest_info = None
        self.is_navigating = False
        self.active_goal_handle = None
        self.goal_start_time = 0.0
        self.current_goal_coord = None
        self.consecutive_empty_frontiers = 0
        self.exploration_completed = False

        # Blacklist: {(gx, gy): fail_count}
        self.blacklist = {}
        # Visited history: [(gx, gy)]
        self.visited_goals = []

        # Periodic exploration loop timer
        self.timer = self.create_timer(self.plan_interval, self.exploration_cycle)

    def map_callback(self, msg: OccupancyGrid):
        """Cache incoming occupancy grid map."""
        self.latest_info = copy.deepcopy(msg.info)
        # Store as 2D array [height, width]
        self.latest_map = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def get_robot_pose(self):
        """Retrieve current robot pose in map frame via TF."""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=0.5)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y
        except Exception:
            # Fallback if map frame is just 'map'
            try:
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    self.base_frame,
                    now,
                    timeout=Duration(seconds=0.2)
                )
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                return x, y
            except Exception:
                return None, None

    def exploration_cycle(self):
        """Main exploration step: check status, extract frontiers, assign goal."""
        if self.exploration_completed:
            return

        if self.latest_map is None or self.latest_info is None:
            return

        # Check for goal timeouts if currently navigating
        if self.is_navigating:
            elapsed = time.time() - self.goal_start_time
            if elapsed > self.max_goal_duration:
                self.get_logger().warn(
                    f'Navigation goal taking too long ({elapsed:.1f}s). Cancelling to replan.'
                )
                self.cancel_current_goal()
            return

        # Find robot position
        rx, ry = self.get_robot_pose()
        if rx is None:
            # TF might not be ready yet
            return

        # Extract reachable frontiers
        frontiers = self.detect_frontiers(self.latest_map, self.latest_info, rx, ry)

        if not frontiers:
            self.consecutive_empty_frontiers += 1
            if self.consecutive_empty_frontiers >= 3:
                self.get_logger().info(
                    f'[{self.ns or "robot"}] All frontiers explored! Autonomous exploration complete.'
                )
                self.exploration_completed = True
            return

        self.consecutive_empty_frontiers = 0

        # Select best frontier using distance, cluster size, and regional bias
        chosen_frontier = self.select_best_frontier(frontiers, rx, ry)
        if chosen_frontier is None:
            return

        target_x, target_y = chosen_frontier
        self.send_navigation_goal(target_x, target_y)

    def detect_frontiers(self, grid, info, rx, ry):
        """
        Morphological frontier detection on occupancy grid.
        Free space (0) dilated into Unknown space (-1).
        """
        h, w = grid.shape
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        # Binary masks
        free_mask = (grid == 0).astype(np.uint8) * 255
        unknown_mask = (grid == -1).astype(np.uint8) * 255
        obstacle_mask = (grid > 50).astype(np.uint8) * 255

        # Safety inflation around obstacles to prevent collision goals
        safety_cells = max(1, int(self.safety_distance / resolution))
        kernel_obs = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (safety_cells * 2 + 1, safety_cells * 2 + 1)
        )
        inflated_obstacles = cv2.dilate(obstacle_mask, kernel_obs)

        # Dilate free space with 3x3 kernel
        kernel_free = np.ones((3, 3), np.uint8)
        dilated_free = cv2.dilate(free_mask, kernel_free, iterations=1)

        # Frontiers are dilated free pixels touching unknown space, excluding obstacles
        raw_frontiers = cv2.bitwise_and(dilated_free, unknown_mask)
        raw_frontiers = cv2.bitwise_and(raw_frontiers, cv2.bitwise_not(inflated_obstacles))

        # Cluster frontier cells using contours
        contours, _ = cv2.findContours(
            raw_frontiers, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            arc_len = cv2.arcLength(cnt, closed=True)
            # Accept if area or length satisfies minimum size
            if area < self.min_frontier_size and arc_len < (self.min_frontier_size * 2):
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Clamp coordinates
            cx = max(0, min(w - 1, cx))
            cy = max(0, min(h - 1, cy))

            # Ensure candidate goal cell is safe (not inside an obstacle)
            if inflated_obstacles[cy, cx] > 0 or grid[cy, cx] > 50:
                # Find nearest point in contour that is safe
                safe_found = False
                for pt in cnt:
                    px, py = pt[0][0], pt[0][1]
                    if inflated_obstacles[py, px] == 0 and grid[py, px] <= 50:
                        cx, cy = px, py
                        safe_found = True
                        break
                if not safe_found:
                    continue

            # Convert to world coordinates
            wx = origin_x + (cx + 0.5) * resolution
            wy = origin_y + (cy + 0.5) * resolution

            # Filter distance from robot
            dist = math.hypot(wx - rx, wy - ry)
            if dist < self.min_goal_distance or dist > self.max_goal_distance:
                continue

            # Check blacklist
            is_blacklisted = False
            for (bx, by), fail_count in self.blacklist.items():
                if math.hypot(wx - bx, wy - by) < 0.6 and fail_count >= 3:
                    is_blacklisted = True
                    break
            if is_blacklisted:
                continue

            candidates.append({
                'coord': (wx, wy),
                'dist': dist,
                'size': max(area, arc_len),
            })

        return candidates

    def select_best_frontier(self, frontiers, rx, ry):
        """
        Frontier ranking utility function.
        Balances cluster information gain, traveling distance, and regional bias.
        """
        best_score = -float('inf')
        best_coord = None

        for f in frontiers:
            wx, wy = f['coord']
            dist = f['dist']
            size = f['size']

            # Visited suppression penalty
            visited_penalty = 1.0
            for vx, vy in self.visited_goals:
                if math.hypot(wx - vx, wy - vy) < 1.0:
                    visited_penalty = 0.3
                    break

            # Regional bias reward
            regional_bonus = 1.0
            if self.bias_y_sign != 0.0:
                # If target is in robot's preferred direction (+y or -y), give bonus
                if (wy * self.bias_y_sign) > 0.0:
                    regional_bonus = 1.5
                else:
                    regional_bonus = 0.8

            # Utility score
            score = (math.sqrt(size) / (dist + 0.5)) * visited_penalty * regional_bonus

            if score > best_score:
                best_score = score
                best_coord = (wx, wy)

        return best_coord

    def send_navigation_goal(self, x, y):
        """Send target frontier pose to Nav2 action server."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 navigate_to_pose action server unavailable.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Calculate yaw toward target from current robot position
        rx, ry = self.get_robot_pose()
        if rx is not None:
            yaw = math.atan2(y - ry, x - rx)
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to frontier: ({x:.2f}, {y:.2f})')
        self.is_navigating = True
        self.goal_start_time = time.time()
        self.current_goal_coord = (x, y)

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection from Nav2 server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2 server.')
            self.register_failure()
            self.is_navigating = False
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        """Optionally process navigation feedback."""
        pass

    def goal_result_callback(self, future):
        """Handle result when navigation completes, aborts, or is canceled."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Goal {self.current_goal_coord} reached successfully!'
            )
            if self.current_goal_coord:
                self.visited_goals.append(self.current_goal_coord)
        else:
            self.get_logger().warn(
                f'Goal failed with status: {status}. Adding to blacklist.'
            )
            self.register_failure()

        self.is_navigating = False
        self.active_goal_handle = None

    def cancel_current_goal(self):
        """Cancel active navigation goal."""
        if self.active_goal_handle is not None:
            self.get_logger().info('Cancelling active goal...')
            self.active_goal_handle.cancel_goal_async()
        self.register_failure()
        self.is_navigating = False
        self.active_goal_handle = None

    def register_failure(self):
        """Register goal failure in blacklist to prevent repeated attempts."""
        if self.current_goal_coord:
            gx, gy = self.current_goal_coord
            # Key rounded to 0.5m grid
            key = (round(gx * 2) / 2, round(gy * 2) / 2)
            self.blacklist[key] = self.blacklist.get(key, 0) + 1


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Frontier explorer stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
