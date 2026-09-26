#!/usr/bin/env python3
"""
Autonomous Frontier Exploration Node with Nav2 Integration
- Identifies contiguous frontier boundaries between explored and unknown space.
- Clusters contiguous frontier cells and computes the exact center (centroid) of each boundary.
- Selects the center of the NEAREST valid frontier boundary to the robot.
- Uses Nav2 (NavigateToPose action & goal_pose topic) to direct the robot to the frontier center.
- Provides automatic replanning on goal arrival and blacklists unreachable frontiers.
- Includes stuck detection (moved < 5cm in 7s) with escape maneuvers opposite to the nearest obstacle.
"""

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def yaw_to_quaternion(yaw: float):
    """Convert 2D yaw angle (radians) to quaternion (x, y, z, w)."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration')

        # Parameters
        self.declare_parameter('robot_namespace', 'robot1')
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('linear_speed', 0.22)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('obstacle_dist', 0.65)
        self.declare_parameter('goal_lock_duration', 6.0)
        self.declare_parameter('stuck_timeout', 7.0)
        self.declare_parameter('stuck_dist_threshold', 0.05)

        node_ns = self.get_namespace().strip('/')
        raw_param = self.get_parameter('robot_namespace').get_parameter_value().string_value.strip('/')
        self.namespace = raw_param if raw_param else node_ns
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value
        self.max_linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.obstacle_dist = self.get_parameter('obstacle_dist').get_parameter_value().double_value
        self.goal_lock_duration = self.get_parameter('goal_lock_duration').get_parameter_value().double_value
        self.stuck_timeout = self.get_parameter('stuck_timeout').get_parameter_value().double_value
        self.stuck_dist_threshold = self.get_parameter('stuck_dist_threshold').get_parameter_value().double_value

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot State in Map Frame
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_valid = False

        # Sensor data
        self.latest_map = None
        self.laser_ranges = None
        self.laser_angles = None

        # Goal and Nav2 State
        self.current_goal = None
        self.current_goal_yaw = 0.0
        self.last_goal_time = None
        self.replan_needed = True
        self.nav_goal_active = False
        self.current_goal_handle = None
        self.unreachable_goals = []

        # Stuck Recovery State Machine (moved < 5cm in 7s)
        self.escape_mode = False
        self.escape_timer = 0
        self.escape_heading = 0.0
        self.last_pos = (0.0, 0.0)
        self.stuck_check_time = None

        # QoS Profiles
        map_qos = QoSProfile(
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Topics
        map_topic = f'/{self.namespace}/map' if self.namespace else '/map'
        scan_topic = f'/{self.namespace}/scan' if self.namespace else '/scan'
        odom_topic = f'/{self.namespace}/odom' if self.namespace else '/odom'
        cmd_vel_topic = f'/{self.namespace}/cmd_vel' if self.namespace else '/cmd_vel'
        goal_viz_topic = f'/{self.namespace}/current_frontier' if self.namespace else '/current_frontier'
        goal_pose_topic = f'/{self.namespace}/goal_pose' if self.namespace else '/goal_pose'
        nav_action_name = f'/{self.namespace}/navigate_to_pose' if self.namespace else '/navigate_to_pose'

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.goal_viz_pub = self.create_publisher(PointStamped, goal_viz_topic, 10)
        self.nav2_goal_pub = self.create_publisher(PoseStamped, goal_pose_topic, 10)

        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, nav_action_name)

        # 10 Hz Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Nav2 Frontier Exploration initialized for [{self.namespace}] targeting center of nearest frontier'
        )

    def update_pose_from_tf(self):
        """Fetch robot pose in map coordinate frame."""
        map_frame = f'{self.namespace}/map' if self.namespace else 'map'
        base_frame = f'{self.namespace}/base_footprint' if self.namespace else 'base_footprint'

        try:
            t = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.03)
            )
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.pose_valid = True
            return True
        except Exception:
            return False

    def odom_callback(self, msg: Odometry):
        if not self.pose_valid:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if self.laser_angles is None or len(self.laser_angles) != n:
            self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, n)

        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), msg.range_max, ranges)
        self.laser_ranges = ranges

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def extract_frontier_centers(self):
        """
        Extract contiguous frontier boundaries between explored and unknown regions.
        Clusters contiguous frontier cells and computes the exact center (centroid) of each boundary.
        """
        if self.latest_map is None:
            return []

        info = self.latest_map.info
        width = info.width
        height = info.height
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        grid = np.array(self.latest_map.data, dtype=np.int8).reshape((height, width))
        occupied = (grid > 50)

        # 1-cell wall gap filling
        vert_gap = np.zeros_like(occupied, dtype=bool)
        vert_gap[1:-1, :] = occupied[:-2, :] & occupied[2:, :]
        horiz_gap = np.zeros_like(occupied, dtype=bool)
        horiz_gap[:, 1:-1] = occupied[:, :-2] & occupied[:, 2:]
        diag1 = np.zeros_like(occupied, dtype=bool)
        diag1[1:-1, 1:-1] = occupied[:-2, :-2] & occupied[2:, 2:]
        diag2 = np.zeros_like(occupied, dtype=bool)
        diag2[1:-1, 1:-1] = occupied[:-2, 2:] & occupied[2:, :-2]
        occupied = occupied | vert_gap | horiz_gap | diag1 | diag2

        free = (grid == 0) & (~occupied)
        unknown = (grid == -1) & (~occupied)

        # 4-connected neighbor check for unknown cells
        adj_unknown = np.zeros_like(free, dtype=bool)
        adj_unknown[:-1, :] |= unknown[1:, :]
        adj_unknown[1:, :] |= unknown[:-1, :]
        adj_unknown[:, :-1] |= unknown[:, 1:]
        adj_unknown[:, 1:] |= unknown[:, :-1]

        frontiers = free & adj_unknown

        # Safety buffer around occupied walls (~25cm = 5 cells at 0.05m)
        near_wall = np.zeros_like(occupied, dtype=bool)
        wall_buffer = occupied.copy()
        for _ in range(5):
            near_wall[:-1, :] |= wall_buffer[1:, :]
            near_wall[1:, :] |= wall_buffer[:-1, :]
            near_wall[:, :-1] |= wall_buffer[:, 1:]
            near_wall[:, 1:] |= wall_buffer[:, :-1]
            wall_buffer = near_wall.copy()

        safe_frontiers = frontiers & (~near_wall)

        # Cluster contiguous frontier cells using 8-connectivity
        num_labels, labels = cv2.connectedComponents(safe_frontiers.astype(np.uint8), connectivity=8)

        frontier_centers = []
        for lbl in range(1, num_labels):
            rows, cols = np.where(labels == lbl)
            if len(rows) < self.min_frontier_size:
                continue

            # Compute centroid of this contiguous frontier boundary
            mean_c = np.mean(cols)
            mean_r = np.mean(rows)

            # Choose the actual frontier cell closest to the centroid to guarantee it lies on a free cell
            dists_sq = (cols - mean_c)**2 + (rows - mean_r)**2
            best_idx = np.argmin(dists_sq)

            cx = origin_x + (cols[best_idx] + 0.5) * res
            cy = origin_y + (rows[best_idx] + 0.5) * res

            frontier_centers.append({
                'x': float(cx),
                'y': float(cy),
                'size': len(rows)
            })

        return frontier_centers

    def select_nearest_frontier_center(self, frontier_centers):
        """
        Select the center of the NEAREST valid frontier boundary to the robot.
        """
        if not frontier_centers:
            return None

        min_dist_threshold = 0.50
        candidates = []

        for fc in frontier_centers:
            dist = math.hypot(fc['x'] - self.robot_x, fc['y'] - self.robot_y)
            if dist < min_dist_threshold:
                continue

            # Check if this frontier was marked as unreachable
            is_unreachable = False
            for (ux, uy) in self.unreachable_goals:
                if math.hypot(fc['x'] - ux, fc['y'] - uy) < 0.60:
                    is_unreachable = True
                    break

            if not is_unreachable:
                candidates.append((dist, fc))

        # If all candidates were blacklisted, clear blacklist and re-evaluate
        if not candidates:
            if self.unreachable_goals:
                self.unreachable_goals.clear()
                for fc in frontier_centers:
                    dist = math.hypot(fc['x'] - self.robot_x, fc['y'] - self.robot_y)
                    if dist >= min_dist_threshold:
                        candidates.append((dist, fc))

        if not candidates:
            return None

        # Sort strictly by distance to find the NEAREST frontier boundary center
        candidates.sort(key=lambda item: item[0])
        nearest_dist, nearest_fc = candidates[0]

        # Desired heading points from robot towards the frontier center
        target_yaw = math.atan2(nearest_fc['y'] - self.robot_y, nearest_fc['x'] - self.robot_x)

        return nearest_fc['x'], nearest_fc['y'], target_yaw

    def send_nav2_goal(self, target_x: float, target_y: float, target_yaw: float):
        """
        Direct the robot to the frontier center using Nav2.
        Publishes PoseStamped and PointStamped, and triggers NavigateToPose action.
        """
        now = self.get_clock().now()
        map_frame = f'{self.namespace}/map' if self.namespace else 'map'

        # 1. PoseStamped for Nav2 goal_pose and RViz 2D Goal Display
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = now.to_msg()
        goal_pose_msg.header.frame_id = map_frame
        goal_pose_msg.pose.position.x = float(target_x)
        goal_pose_msg.pose.position.y = float(target_y)
        goal_pose_msg.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(target_yaw)
        goal_pose_msg.pose.orientation.x = qx
        goal_pose_msg.pose.orientation.y = qy
        goal_pose_msg.pose.orientation.z = qz
        goal_pose_msg.pose.orientation.w = qw
        self.nav2_goal_pub.publish(goal_pose_msg)

        # 2. PointStamped for current frontier visualization in RViz
        p = PointStamped()
        p.header.stamp = now.to_msg()
        p.header.frame_id = map_frame
        p.point.x = float(target_x)
        p.point.y = float(target_y)
        p.point.z = 0.0
        self.goal_viz_pub.publish(p)

        # 3. Nav2 NavigateToPose Action
        if self.nav_to_pose_client.server_is_ready():
            if self.nav_goal_active and self.current_goal_handle is not None:
                self.get_logger().info(f'[{self.namespace}] Cancelling previous goal for new nearest frontier...')
                self.current_goal_handle.cancel_goal_async()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose_msg

            self.get_logger().info(
                f'Nav2 directing [{self.namespace}] to center of nearest frontier at ({target_x:.2f}, {target_y:.2f})'
            )
            self.nav_goal_active = True
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav_goal_response_callback)
        else:
            self.get_logger().debug(
                f'Nav2 action server not connected yet for [{self.namespace}]. Published goal to /{self.namespace}/goal_pose.'
            )

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Nav2 goal for [{self.namespace}] rejected by server!')
            self.handle_nav_failure()
            return

        self.get_logger().info(f'Nav2 goal for [{self.namespace}] accepted. Navigating...')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, gh=goal_handle: self.nav_goal_result_callback(fut, gh)
        )

    def nav_goal_result_callback(self, future, goal_handle):
        # Ignore result if this callback belongs to an old superseded goal handle
        if self.current_goal_handle is not None and self.current_goal_handle != goal_handle:
            self.get_logger().debug(f'Ignoring result from previous superseded goal for [{self.namespace}].')
            return

        self.nav_goal_active = False
        self.current_goal_handle = None
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Robot [{self.namespace}] arrived at nearest frontier center!')
            self.current_goal = None
            self.replan_needed = True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Nav2 goal for [{self.namespace}] was canceled for replanning.')
        else:
            self.get_logger().warn(f'Nav2 goal for [{self.namespace}] finished with status {status}. Blacklisting.')
            self.handle_nav_failure()

    def handle_nav_failure(self):
        if self.current_goal is not None:
            self.unreachable_goals.append((self.current_goal[0], self.current_goal[1]))
            if len(self.unreachable_goals) > 10:
                self.unreachable_goals.pop(0)
            self.current_goal = None
        self.nav_goal_active = False
        self.current_goal_handle = None
        self.replan_needed = True

    def control_loop(self):
        self.update_pose_from_tf()

        if self.laser_ranges is None:
            return

        now = self.get_clock().now()
        twist = Twist()

        # --- Stuck / Circling Detection (moved < 5cm in 7 seconds) ---
        if self.stuck_check_time is None:
            self.stuck_check_time = now
            self.last_pos = (self.robot_x, self.robot_y)
        else:
            dt_stuck = (now - self.stuck_check_time).nanoseconds / 1e9
            if dt_stuck >= self.stuck_timeout:
                dist_traveled = math.hypot(self.robot_x - self.last_pos[0], self.robot_y - self.last_pos[1])
                if dist_traveled < self.stuck_dist_threshold:
                    self.get_logger().warn(
                        f'Stuck detected (moved {dist_traveled:.3f}m < {self.stuck_dist_threshold}m in {dt_stuck:.1f}s)! Escaping opposite to nearest obstacle...'
                    )
                    # Cancel active Nav2 goal if any
                    if self.nav_goal_active and self.current_goal_handle is not None:
                        self.current_goal_handle.cancel_goal_async()
                    self.handle_nav_failure()

                    self.escape_mode = True
                    self.escape_timer = 25  # 2.5 seconds at 10 Hz

                    # Find direction opposite to nearest obstacle
                    if self.laser_ranges is not None and self.laser_angles is not None:
                        valid_mask = (self.laser_ranges > 0.05) & (self.laser_ranges < 3.0)
                        if np.any(valid_mask):
                            closest_idx = np.argmin(self.laser_ranges[valid_mask])
                            nearest_angle = self.laser_angles[valid_mask][closest_idx]
                            away_angle = nearest_angle + math.pi
                            while away_angle > math.pi:
                                away_angle -= 2 * math.pi
                            while away_angle < -math.pi:
                                away_angle += 2 * math.pi
                            self.escape_heading = away_angle
                        else:
                            self.escape_heading = math.pi
                    else:
                        self.escape_heading = math.pi

                self.stuck_check_time = now
                self.last_pos = (self.robot_x, self.robot_y)

        # --- Escape Maneuver Execution (Directly Opposite to Nearest Obstacle) ---
        if self.escape_mode:
            self.escape_timer -= 1
            if abs(self.escape_heading) > math.radians(135):
                if self.escape_timer > 10:
                    twist.linear.x = -0.12
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = self.max_angular_speed if self.escape_heading >= 0 else -self.max_angular_speed
            else:
                twist.linear.x = 0.12
                twist.angular.z = np.clip(self.escape_heading * 1.5, -self.max_angular_speed, self.max_angular_speed)

            if self.escape_timer <= 0:
                self.escape_mode = False
                self.get_logger().info('Escape complete. Resuming Nav2 frontier exploration.')
                self.replan_needed = True

            self.cmd_vel_pub.publish(twist)
            return

        # --- Frontier Selection: Center of Nearest Frontier ---
        should_replan = self.replan_needed or (self.current_goal is None)
        if not should_replan and self.last_goal_time is not None:
            time_elapsed = (now - self.last_goal_time).nanoseconds / 1e9
            dist_to_goal = math.hypot(self.current_goal[0] - self.robot_x, self.current_goal[1] - self.robot_y)
            if time_elapsed >= self.goal_lock_duration or dist_to_goal < 0.45:
                should_replan = True

        if should_replan:
            frontier_centers = self.extract_frontier_centers()
            target = self.select_nearest_frontier_center(frontier_centers)
            if target is not None:
                target_x, target_y, target_yaw = target
                # Check if target is sufficiently different from current
                if self.current_goal is None or math.hypot(target_x - self.current_goal[0], target_y - self.current_goal[1]) > 0.40:
                    self.current_goal = (target_x, target_y)
                    self.current_goal_yaw = target_yaw
                    self.last_goal_time = now
                    self.replan_needed = False
                    self.send_nav2_goal(target_x, target_y, target_yaw)
            else:
                self.current_goal = None

        # --- Fallback Reactive Guidance (only if Nav2 action server is not active) ---
        if not self.nav_goal_active:
            if self.current_goal is not None:
                gx, gy = self.current_goal
                vx_map = gx - self.robot_x
                vy_map = gy - self.robot_y
                g_dist = math.hypot(vx_map, vy_map)
                vx_att = (vx_map * math.cos(self.robot_yaw) + vy_map * math.sin(self.robot_yaw)) / max(g_dist, 0.01)
                vy_att = (-vx_map * math.sin(self.robot_yaw) + vy_map * math.cos(self.robot_yaw)) / max(g_dist, 0.01)
            else:
                vx_att, vy_att = 1.0, 0.0

            vx_rep = 0.0
            vy_rep = 0.0
            min_front_dist = 3.5

            for r, a in zip(self.laser_ranges, self.laser_angles):
                while a > math.pi:
                    a -= 2 * math.pi
                while a < -math.pi:
                    a += 2 * math.pi

                if abs(a) < math.radians(35):
                    if r < min_front_dist:
                        min_front_dist = r

                if abs(a) < math.radians(100) and r < self.obstacle_dist:
                    force = (self.obstacle_dist - r) / max(r, 0.08)
                    vx_rep -= force * math.cos(a)
                    vy_rep -= force * math.sin(a)

            if min_front_dist < 0.32:
                twist.linear.x = -0.10
                twist.angular.z = self.max_angular_speed * (1.0 if vy_rep >= 0 else -1.0)
                self.cmd_vel_pub.publish(twist)
                return

            rep_weight = 1.5
            vx_total = vx_att + rep_weight * vx_rep
            vy_total = vy_att + rep_weight * vy_rep
            desired_heading = math.atan2(vy_total, vx_total)

            heading_error = abs(desired_heading)
            if heading_error > 0.8:
                twist.linear.x = 0.04
                twist.angular.z = np.clip(desired_heading * 1.5, -self.max_angular_speed, self.max_angular_speed)
            else:
                speed_factor = max(0.2, (1.0 - heading_error / 0.8))
                obs_factor = max(0.3, min(1.0, (min_front_dist - 0.35) / 0.30))
                twist.linear.x = self.max_linear_speed * speed_factor * obs_factor
                twist.angular.z = np.clip(desired_heading * 1.2, -self.max_angular_speed, self.max_angular_speed)

            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
