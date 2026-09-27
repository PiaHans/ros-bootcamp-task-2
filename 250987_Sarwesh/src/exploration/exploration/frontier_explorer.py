import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from scipy.ndimage import binary_dilation


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('min_frontier_size', 4)
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value

        self.get_logger().info(f'Starting Frontier Explorer for {self.robot_name}...')

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, f'/{self.robot_name}/navigate_to_pose')

        # Map Subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            f'/{self.robot_name}/map',
            self.map_callback,
            10
        )

        self.map_data = None
        self.is_navigating = False
        self.visited_goals = []
        self.current_goal = None
        self.goal_start_time = None
        self.completed = False

        # Main Exploration Loop
        self.timer = self.create_timer(2.0, self.exploration_loop)

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                f'{self.robot_name}/map',
                f'{self.robot_name}/base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            return (trans.transform.translation.x, trans.transform.translation.y)
        except Exception:
            return None

    def find_frontiers(self):
        if self.map_data is None:
            return []

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        grid = np.array(self.map_data.data, dtype=np.int8).reshape((height, width))

        free_mask = (grid == 0)
        unknown_mask = (grid == -1)
        obstacle_mask = (grid > 50)

        # Inflate obstacles with scipy.ndimage
        inflated_obstacles = binary_dilation(obstacle_mask, iterations=3)

        # Find free cells next to unknown space
        unknown_padded = np.pad(unknown_mask, 1, mode='constant', constant_values=False)
        has_unknown_neighbor = (
            unknown_padded[:-2, 1:-1] | unknown_padded[2:, 1:-1] |
            unknown_padded[1:-1, :-2] | unknown_padded[1:-1, 2:]
        )

        frontier_mask = free_mask & has_unknown_neighbor & (~inflated_obstacles)
        frontier_indices = np.argwhere(frontier_mask)
        if len(frontier_indices) == 0:
            return []

        visited = set()
        clusters = []

        for y, x in frontier_indices:
            if (y, x) in visited:
                continue

            cluster = []
            queue = [(y, x)]
            visited.add((y, x))

            while queue:
                cy, cx = queue.pop(0)
                cluster.append((cx * resolution + origin_x, cy * resolution + origin_y))

                for dy, dx in [(-1,0), (1,0), (0,-1), (0,1)]:
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        if frontier_mask[ny, nx] and (ny, nx) not in visited:
                            visited.add((ny, nx))
                            queue.append((ny, nx))

            if len(cluster) >= self.min_frontier_size:
                mean_x = float(np.mean([p[0] for p in cluster]))
                mean_y = float(np.mean([p[1] for p in cluster]))
                clusters.append((mean_x, mean_y))

        return clusters

    def exploration_loop(self):
        if self.completed:
            return

        # Handle goal timeout (re-plan if navigating for > 35 seconds)
        if self.is_navigating and self.goal_start_time:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed > 35.0:
                self.get_logger().warn(f'[{self.robot_name}] Goal timed out. Re-planning...')
                if self.current_goal:
                    self.visited_goals.append(self.current_goal)
                self.is_navigating = False
                return

        if self.is_navigating:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().info(f'[{self.robot_name}] Waiting for robot pose transform...')
            return

        rx, ry = robot_pose
        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info(f'[{self.robot_name}] No frontiers detected. Checking completion...')
            if self.map_data is not None and len(self.visited_goals) > 2:
                self.completed = True
                self.get_logger().info(f'🎉 [{self.robot_name}] Autonomous exploration complete! Whole area mapped.')
            return

        # 1. Filter out frontiers closer than 0.6m to current position (robot is already there!)
        # 2. Filter out already visited / blacklisted frontiers
        valid_frontiers = []
        for fx, fy in frontiers:
            dist_to_robot = math.hypot(fx - rx, fy - ry)
            if dist_to_robot < 0.6:
                continue

            already_visited = False
            for vx, vy in self.visited_goals:
                if math.hypot(fx - vx, fy - vy) < 0.4:
                    already_visited = True
                    break

            if not already_visited:
                valid_frontiers.append((fx, fy, dist_to_robot))

        if not valid_frontiers:
            self.get_logger().info(f'[{self.robot_name}] All reachable frontiers explored. Mission complete!')
            self.completed = True
            return

        # Sort by distance from robot (nearest frontier first)
        valid_frontiers.sort(key=lambda item: item[2])
        best_goal = (valid_frontiers[0][0], valid_frontiers[0][1])

        self.send_goal(best_goal)

    def send_goal(self, goal_xy):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f'[{self.robot_name}] Nav2 action server not ready yet.')
            return

        goal_x, goal_y = goal_xy
        self.current_goal = goal_xy
        self.is_navigating = True
        self.goal_start_time = self.get_clock().now()

        robot_pose = self.get_robot_pose()
        yaw = 0.0
        if robot_pose:
            yaw = math.atan2(goal_y - robot_pose[1], goal_x - robot_pose[0])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = f'{self.robot_name}/map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        dist = math.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1]) if robot_pose else 0.0
        self.get_logger().info(f'[{self.robot_name}] 🚀 Driving to frontier at ({goal_x:.2f}, {goal_y:.2f}) [distance: {dist:.2f}m]')

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{self.robot_name}] Goal was rejected by Nav2.')
            if self.current_goal:
                self.visited_goals.append(self.current_goal)
            self.is_navigating = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # Status 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info(f'[{self.robot_name}] ✅ Reached frontier! Searching for next unknown area...')
        else:
            self.get_logger().warn(f'[{self.robot_name}] Navigation ended with status {status}. Moving to next frontier...')

        if self.current_goal:
            self.visited_goals.append(self.current_goal)

        self.is_navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
