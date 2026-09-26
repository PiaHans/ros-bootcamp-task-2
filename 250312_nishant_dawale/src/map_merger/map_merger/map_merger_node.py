#!/usr/bin/env python3

import copy
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapMerger(Node):
    """
    Multi-Robot Map Merger & Stitching Node.
    Subscribes to independent local occupancy grids (/robot1/map, /robot2/map),
    performs ORB feature detection & RANSAC registration for overlap stitching,
    incorporates initial pose transformation priors as fallback,
    and fuses the occupancy probabilities into a unified global map on /global_map.
    """

    def __init__(self):
        super().__init__('map_merger')

        # Parameters for initial robot spawn coordinates (used for pose-prior fallback)
        self.declare_parameter('robot1_x', -2.5)
        self.declare_parameter('robot1_y', 0.0)
        self.declare_parameter('robot1_yaw', 0.0)
        self.declare_parameter('robot2_x', 2.5)
        self.declare_parameter('robot2_y', 0.0)
        self.declare_parameter('robot2_yaw', 3.14159)
        self.declare_parameter('merge_frequency', 1.0)
        self.declare_parameter('feature_matching_min_inliers', 8)

        self.r1_x = self.get_parameter('robot1_x').value
        self.r1_y = self.get_parameter('robot1_y').value
        self.r1_yaw = self.get_parameter('robot1_yaw').value
        self.r2_x = self.get_parameter('robot2_x').value
        self.r2_y = self.get_parameter('robot2_y').value
        self.r2_yaw = self.get_parameter('robot2_yaw').value
        self.min_inliers = self.get_parameter('feature_matching_min_inliers').value
        merge_freq = self.get_parameter('merge_frequency').value

        # Calculate initial relative 2D transformation matrix from Robot 2 to Robot 1
        self.init_affine_matrix = self.compute_initial_relative_transform()

        # Cache for maps
        self.map1 = None
        self.info1 = None
        self.map2 = None
        self.info2 = None

        # ORB Detector
        self.orb = cv2.ORB_create(nfeatures=1500, fastThreshold=10)
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Persistent best estimated transform from online feature matching
        self.online_affine = None
        self.best_inliers = 0

        # Subscribers
        self.sub_map1 = self.create_subscription(
            OccupancyGrid, '/robot1/map', self.map1_callback, 10
        )
        self.sub_map2 = self.create_subscription(
            OccupancyGrid, '/robot2/map', self.map2_callback, 10
        )

        # Publisher
        self.pub_global_map = self.create_publisher(OccupancyGrid, '/global_map', 10)

        # Merge loop timer
        self.timer = self.create_timer(1.0 / max(0.1, merge_freq), self.merge_and_publish)

        self.get_logger().info(
            f'Map Merger initialized. Prior offset TB1->TB2: '
            f'dx={self.r2_x - self.r1_x:.2f}, dy={self.r2_y - self.r1_y:.2f}'
        )

    def compute_initial_relative_transform(self):
        """
        Compute 2x3 affine matrix representing transform from robot2 map frame
        to robot1 map frame based on initial spawn configuration.
        """
        # Relative translation and yaw in world
        # T_w_1: pos = (x1, y1), yaw = yaw1
        # T_w_2: pos = (x2, y2), yaw = yaw2
        # T_1_2 = T_w_1^-1 * T_w_2
        cos1 = math.cos(self.r1_yaw)
        sin1 = math.sin(self.r1_yaw)
        dx_w = self.r2_x - self.r1_x
        dy_w = self.r2_y - self.r1_y

        # Rotate world delta into robot1 frame
        dx_1 = cos1 * dx_w + sin1 * dy_w
        dy_1 = -sin1 * dx_w + cos1 * dy_w
        dyaw = self.r2_yaw - self.r1_yaw

        cos_rel = math.cos(dyaw)
        sin_rel = math.sin(dyaw)

        # 2x3 matrix: [ [R00, R01, tx], [R10, R11, ty] ]
        return np.array([
            [cos_rel, -sin_rel, dx_1],
            [sin_rel, cos_rel, dy_1]
        ], dtype=np.float64)

    def map1_callback(self, msg: OccupancyGrid):
        self.info1 = copy.deepcopy(msg.info)
        self.map1 = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def map2_callback(self, msg: OccupancyGrid):
        self.info2 = copy.deepcopy(msg.info)
        self.map2 = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def grid_to_image(self, grid):
        """Convert occupancy grid to 8-bit grayscale image for ORB."""
        img = np.full(grid.shape, 127, dtype=np.uint8)  # Unknown = 127
        img[grid == 0] = 255                            # Free = 255
        img[grid > 50] = 0                              # Occupied = 0
        return img

    def attempt_feature_registration(self, img1, img2, info1, info2):
        """
        Detect ORB keypoints and find rigid 2D transform via RANSAC.
        Returns: 2x3 affine matrix in metric world coordinates, or None.
        """
        kp1, des1 = self.orb.detectAndCompute(img1, None)
        kp2, des2 = self.orb.detectAndCompute(img2, None)

        if des1 is None or des2 is None or len(kp1) < 8 or len(kp2) < 8:
            return None, 0

        matches = self.bf_matcher.match(des2, des1)  # query=robot2, train=robot1
        if len(matches) < 8:
            return None, 0

        matches = sorted(matches, key=lambda m: m.distance)
        good_matches = matches[:min(len(matches), 50)]

        # Convert pixel points to metric map coordinates
        pts2_world = []
        pts1_world = []

        for m in good_matches:
            # Robot 2 point
            px2, py2 = kp2[m.queryIdx].pt
            wx2 = info2.origin.position.x + px2 * info2.resolution
            wy2 = info2.origin.position.y + py2 * info2.resolution
            pts2_world.append([wx2, wy2])

            # Robot 1 point
            px1, py1 = kp1[m.trainIdx].pt
            wx1 = info1.origin.position.x + px1 * info1.resolution
            wy1 = info1.origin.position.y + py1 * info1.resolution
            pts1_world.append([wx1, wy1])

        pts2_world = np.float32(pts2_world).reshape(-1, 1, 2)
        pts1_world = np.float32(pts1_world).reshape(-1, 1, 2)

        # Estimate partial affine (rotation + translation + uniform scale)
        affine_mat, inliers = cv2.estimateAffinePartial2D(
            pts2_world, pts1_world, method=cv2.RANSAC, ransacReprojThreshold=0.25
        )

        num_inliers = int(np.sum(inliers)) if inliers is not None else 0

        if affine_mat is not None and num_inliers >= self.min_inliers:
            # Check scale consistency (scale should be ~1.0 since both resolutions are equal)
            scale = math.hypot(affine_mat[0, 0], affine_mat[0, 1])
            if 0.85 <= scale <= 1.15:
                # Normalize scale to exact 1.0 to preserve rigid metric geometry
                affine_mat[:2, :2] /= scale
                return affine_mat, num_inliers

        return None, num_inliers

    def merge_and_publish(self):
        """Main periodic map merging pipeline."""
        if self.map1 is None or self.map2 is None:
            return
        if self.info1 is None or self.info2 is None:
            return

        try:
            img1 = self.grid_to_image(self.map1)
            img2 = self.grid_to_image(self.map2)

            # Try online feature matching
            online_mat, inliers = self.attempt_feature_registration(
                img1, img2, self.info1, self.info2
            )

            if online_mat is not None and inliers > self.best_inliers:
                self.online_affine = online_mat
                self.best_inliers = inliers
                self.get_logger().info(
                    f'Feature-based alignment updated with {inliers} inliers!'
                )

            # Select transformation: online feature match if available, else geometric prior
            if self.online_affine is not None:
                T_world = self.online_affine
                alignment_type = f'ORB-RANSAC ({self.best_inliers} inliers)'
            else:
                T_world = self.init_affine_matrix
                alignment_type = 'Geometric Spawn Prior'

            # Build unified global occupancy grid
            merged_grid, global_info = self.stitch_grids(
                self.map1, self.info1, self.map2, self.info2, T_world
            )

            # Publish result
            merged_msg = OccupancyGrid()
            merged_msg.header.stamp = self.get_clock().now().to_msg()
            merged_msg.header.frame_id = 'robot1/map'
            merged_msg.info = global_info
            merged_msg.data = merged_grid.flatten().tolist()

            self.pub_global_map.publish(merged_msg)
            self.get_logger().debug(
                f'Published /global_map ({global_info.width}x{global_info.height}) using {alignment_type}'
            )

        except Exception as e:
            self.get_logger().error(f'Map merge failed: {e}')

    def stitch_grids(self, grid1, info1, grid2, info2, T_world):
        """
        Transform grid2 by T_world (in metric space) and fuse into grid1's coordinate system.
        """
        res = info1.resolution

        # Bounds of map 1 in world coords
        h1, w1 = grid1.shape
        x_min1 = info1.origin.position.x
        y_min1 = info1.origin.position.y
        x_max1 = x_min1 + w1 * res
        y_max1 = y_min1 + h1 * res

        # 4 corners of map 2
        h2, w2 = grid2.shape
        c2_x = [info2.origin.position.x, info2.origin.position.x + w2 * res]
        c2_y = [info2.origin.position.y, info2.origin.position.y + h2 * res]
        corners2 = np.array([
            [c2_x[0], c2_y[0]],
            [c2_x[1], c2_y[0]],
            [c2_x[1], c2_y[1]],
            [c2_x[0], c2_y[1]],
        ], dtype=np.float64)

        # Transform corners of map 2 into map 1 frame
        corners2_h = np.hstack([corners2, np.ones((4, 1))])
        transformed_corners = (T_world @ corners2_h.T).T

        # Unified bounding box
        x_min = min(x_min1, np.min(transformed_corners[:, 0])) - 0.5
        y_min = min(y_min1, np.min(transformed_corners[:, 1])) - 0.5
        x_max = max(x_max1, np.max(transformed_corners[:, 0])) + 0.5
        y_max = max(y_max1, np.max(transformed_corners[:, 1])) + 0.5

        # Dimensions of global grid
        global_w = int(math.ceil((x_max - x_min) / res))
        global_h = int(math.ceil((y_max - y_min) / res))

        # Clamp max size for performance
        global_w = min(1200, max(100, global_w))
        global_h = min(1200, max(100, global_h))

        # Initialize global grid to -1 (unknown)
        global_grid = np.full((global_h, global_w), -1, dtype=np.int8)

        # Paste Map 1 directly
        off_x1 = int(round((x_min1 - x_min) / res))
        off_y1 = int(round((y_min1 - y_min) / res))

        # Safe bounds placement
        gw1 = min(w1, global_w - off_x1)
        gh1 = min(h1, global_h - off_y1)
        if gw1 > 0 and gh1 > 0 and off_x1 >= 0 and off_y1 >= 0:
            sub1 = grid1[:gh1, :gw1]
            known1 = (sub1 != -1)
            target_slice = global_grid[off_y1:off_y1 + gh1, off_x1:off_x1 + gw1]
            target_slice[known1] = sub1[known1]

        # Compute pixel-to-pixel affine matrix from grid2 pixels to global_grid pixels
        # Metric coord in map 2: [px2 * res + x_min2, py2 * res + y_min2]
        # Transformed metric coord: T_world @ [wx2, wy2, 1]
        # Pixel in global grid: [(wx_trans - x_min) / res, (wy_trans - y_min) / res]
        T_m2_px = np.array([
            [res, 0.0, info2.origin.position.x],
            [0.0, res, info2.origin.position.y],
            [0.0, 0.0, 1.0]
        ])
        T_w_3x3 = np.vstack([T_world, [0.0, 0.0, 1.0]])
        T_px_global = np.array([
            [1.0 / res, 0.0, -x_min / res],
            [0.0, 1.0 / res, -y_min / res],
            [0.0, 0.0, 1.0]
        ])

        M_pixel = (T_px_global @ T_w_3x3 @ T_m2_px)[:2, :]

        # Warp grid2 float values with INTER_NEAREST
        grid2_float = grid2.astype(np.float32)
        warped2 = cv2.warpAffine(
            grid2_float,
            M_pixel,
            (global_w, global_h),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=-1.0
        )
        warped_grid2 = np.rint(warped2).astype(np.int8)

        # Fuse warped2 into global_grid
        # Free space
        free2 = (warped_grid2 == 0)
        unmapped_or_free = (global_grid == -1) | (global_grid == 0)
        global_grid[free2 & unmapped_or_free] = 0

        # Occupied space (priority: 100 wins)
        occupied2 = (warped_grid2 > 50)
        global_grid[occupied2] = 100

        # Construct global metadata
        global_info = copy.deepcopy(info1)
        global_info.width = global_w
        global_info.height = global_h
        global_info.resolution = res
        global_info.origin.position.x = float(x_min)
        global_info.origin.position.y = float(y_min)
        global_info.origin.position.z = 0.0
        global_info.origin.orientation.x = 0.0
        global_info.origin.orientation.y = 0.0
        global_info.origin.orientation.z = 0.0
        global_info.origin.orientation.w = 1.0

        return global_grid, global_info


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Map merger node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
