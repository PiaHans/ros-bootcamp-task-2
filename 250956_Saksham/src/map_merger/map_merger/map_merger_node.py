#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener

# Hardcoded initial positions (Known Initial Poses - KIP) in Gazebo world coordinates
DEFAULT_INITIAL_POSES = {
    'robot1': {'x': -2.0, 'y': 0.0, 'yaw': 0.0},
    'robot2': {'x': 2.0, 'y': 0.0, 'yaw': 0.0}
}


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion orientation to 2D yaw angle (radians)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class MapMergerNode(Node):
    """
    Known Initial Poses (KIP) Multi-Robot Map Merger:
    1. Holds hardcoded initial positions (KIP) for each robot.
    2. Takes into account each robot's initial origin in its local SLAM map frame.
    3. Transforms cells from the robot's local coordinate frame into unified global coordinates.
    4. Fuses all transformed grids into a single, unified, seamless OccupancyGrid on '/map'.
    5. Broadcasts static TF transforms connecting global 'map' to each robot's map frame.
    """

    def __init__(self):
        super().__init__('map_merger')

        # Parameters
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('merged_frame_id', 'map')
        self.declare_parameter('merged_topic', '/map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('robot1_initial_pose', [-2.0, 0.0, 0.0])
        self.declare_parameter('robot2_initial_pose', [2.0, 0.0, 0.0])

        self.robot_namespaces = list(
            self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        )
        self.merged_frame_id = self.get_parameter('merged_frame_id').get_parameter_value().string_value
        self.merged_topic = self.get_parameter('merged_topic').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Configure Known Initial Poses (KIP) in global coordinates
        self.initial_poses = {}
        for ns in self.robot_namespaces:
            param_name = f'{ns}_initial_pose'
            if self.has_parameter(param_name):
                pose_val = self.get_parameter(param_name).get_parameter_value().double_array_value
                if len(pose_val) >= 3:
                    self.initial_poses[ns] = {'x': pose_val[0], 'y': pose_val[1], 'yaw': pose_val[2]}
                    continue

            # Fallback to hardcoded KIP
            if ns in DEFAULT_INITIAL_POSES:
                self.initial_poses[ns] = dict(DEFAULT_INITIAL_POSES[ns])
            else:
                self.initial_poses[ns] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Each robot's initial origin in its SLAM map frame (Gazebo spawn origin)
        self.start_poses_in_map = {}
        for ns in self.robot_namespaces:
            if ns in DEFAULT_INITIAL_POSES:
                self.start_poses_in_map[ns] = dict(DEFAULT_INITIAL_POSES[ns])
            else:
                self.start_poses_in_map[ns] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        for ns, pose in self.initial_poses.items():
            self.get_logger().info(
                f'Known Initial Pose for [{ns}]: x={pose["x"]:.2f}m, y={pose["y"]:.2f}m, yaw={math.degrees(pose["yaw"]):.1f}deg'
            )

        self.broadcast_static_transforms()
        self.tf_timer = self.create_timer(1.0, self.broadcast_static_transforms)

        # QoS Profiles
        self.map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Map Subscriptions
        self.maps = {}
        self.subs = []
        for ns in self.robot_namespaces:
            self.subscribe_robot_map(ns)

        # Publishers
        self.merged_map_pub = self.create_publisher(OccupancyGrid, self.merged_topic, self.map_qos)
        self.metadata_pub = self.create_publisher(MapMetaData, f'{self.merged_topic}_metadata', self.map_qos)

        # Map fusion timer
        timer_period = 1.0 / max(0.1, self.publish_rate)
        self.timer = self.create_timer(timer_period, self.merge_and_publish)

        self.get_logger().info(
            f'Map Merger ready: fusing [{", ".join(self.robot_namespaces)}] into [{self.merged_topic}]'
        )

    def subscribe_robot_map(self, ns: str):
        topic = f'/{ns}/map'
        sub = self.create_subscription(
            OccupancyGrid,
            topic,
            lambda msg, name=ns: self.map_callback(msg, name),
            self.map_qos
        )
        self.subs.append(sub)
        self.get_logger().info(f'Subscribed to local map: {topic}')

    def map_callback(self, msg: OccupancyGrid, robot_name: str):
        self.maps[robot_name] = msg

    def broadcast_static_transforms(self):
        """
        Broadcast static transforms connecting global 'map' to each robot's local '<ns>/map' frame.
        The transform accounts for both the Known Initial Pose (KIP) in the global frame and the
        robot's initial origin in its SLAM map frame.
        """
        now = self.get_clock().now().to_msg()
        transforms = []
        for ns, kip in self.initial_poses.items():
            start = self.start_poses_in_map.get(ns, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

            # Relative offset between global frame and the robot's local map frame
            dx = float(kip['x'] - start['x'])
            dy = float(kip['y'] - start['y'])
            dyaw = float(kip['yaw'] - start['yaw'])

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.merged_frame_id
            t.child_frame_id = f'{ns}/map'
            t.transform.translation.x = dx
            t.transform.translation.y = dy
            t.transform.translation.z = 0.0

            half_yaw = dyaw * 0.5
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(half_yaw)
            t.transform.rotation.w = math.cos(half_yaw)
            transforms.append(t)

        self.static_tf_broadcaster.sendTransform(transforms)

    def merge_and_publish(self):
        """
        Transform all local occupancy grids to the global frame and fuse into a unified map.
        """
        if not self.maps:
            return

        res = self.resolution

        # 1. Calculate the global bounding box from all active robot maps
        all_gx = []
        all_gy = []

        for ns, msg in self.maps.items():
            kip = self.initial_poses.get(ns, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
            start = self.start_poses_in_map.get(ns, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            w = msg.info.width
            h = msg.info.height
            m_res = msg.info.resolution

            ori_q = msg.info.origin.orientation
            ori_yaw = quaternion_to_yaw(ori_q.x, ori_q.y, ori_q.z, ori_q.w)
            cos_o, sin_o = math.cos(ori_yaw), math.sin(ori_yaw)

            local_corners = [
                (0.0, 0.0),
                (w * m_res, 0.0),
                (0.0, h * m_res),
                (w * m_res, h * m_res)
            ]

            cos_k, sin_k = math.cos(kip['yaw']), math.sin(kip['yaw'])

            for cx, cy in local_corners:
                # Coordinate in robot map frame
                lx = ox + (cx * cos_o - cy * sin_o)
                ly = oy + (cx * sin_o + cy * cos_o)

                # Local coordinate relative to robot's initial position
                loc_x = lx - start['x']
                loc_y = ly - start['y']

                # Global coordinate via KIP
                gx = kip['x'] + (loc_x * cos_k - loc_y * sin_k)
                gy = kip['y'] + (loc_x * sin_k + loc_y * cos_k)
                all_gx.append(gx)
                all_gy.append(gy)

        if not all_gx:
            return

        # Stable canvas boundaries: cover at least the 10x10 maze arena [-6.0, 6.0] with stable anchor
        min_x = min(-6.0, math.floor(min(all_gx) - 0.5))
        max_x = max(6.0, math.ceil(max(all_gx) + 0.5))
        min_y = min(-6.0, math.floor(min(all_gy) - 0.5))
        max_y = max(6.0, math.ceil(max(all_gy) + 0.5))

        w_glob = int(round((max_x - min_x) / res))
        h_glob = int(round((max_y - min_y) / res))

        if w_glob <= 0 or h_glob <= 0:
            return

        # 2. Initialize unified grid with -1 (Unknown)
        unified_grid = np.full((h_glob, w_glob), -1, dtype=np.int8)

        # 3. Transform and project each robot's local map into global coordinates
        for ns, msg in self.maps.items():
            kip = self.initial_poses.get(ns, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
            start = self.start_poses_in_map.get(ns, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            w = msg.info.width
            h = msg.info.height
            m_res = msg.info.resolution

            ori_q = msg.info.origin.orientation
            ori_yaw = quaternion_to_yaw(ori_q.x, ori_q.y, ori_q.z, ori_q.w)
            cos_o, sin_o = math.cos(ori_yaw), math.sin(ori_yaw)

            local_data = np.array(msg.data, dtype=np.int8).reshape((h, w))

            # Only fuse known cells (free space >= 0 or occupied == 100)
            known_rows, known_cols = np.where(local_data >= 0)
            if len(known_rows) == 0:
                continue

            vals = local_data[known_rows, known_cols]

            # Metric cell coordinates within local map
            cell_u = (known_cols + 0.5) * m_res
            cell_v = (known_rows + 0.5) * m_res

            # Rotate by local origin orientation
            lx = ox + (cell_u * cos_o - cell_v * sin_o)
            ly = oy + (cell_u * sin_o + cell_v * cos_o)

            # Local coordinates relative to the robot's initial position
            loc_x = lx - start['x']
            loc_y = ly - start['y']

            # Transform from robot local coordinate frame to global frame via KIP
            cos_k, sin_k = math.cos(kip['yaw']), math.sin(kip['yaw'])
            gx_metric = kip['x'] + (loc_x * cos_k - loc_y * sin_k)
            gy_metric = kip['y'] + (loc_x * sin_k + loc_y * cos_k)

            # Convert global metric coordinates to unified grid cell indices
            gx_idx = np.floor((gx_metric - min_x) / res).astype(np.int32)
            gy_idx = np.floor((gy_metric - min_y) / res).astype(np.int32)

            # Filter valid cell indices within canvas
            valid = (gx_idx >= 0) & (gx_idx < w_glob) & (gy_idx >= 0) & (gy_idx < h_glob)
            if not np.any(valid):
                continue

            gx_idx = gx_idx[valid]
            gy_idx = gy_idx[valid]
            vals = vals[valid]

            # Vectorized fusion:
            # -1 (unknown) + known -> known
            # 0 (free) + 0 (free) -> 0
            # 0 (free) + 100 (obstacle) -> 100 (obstacle takes precedence)
            # 100 (obstacle) + 100 (obstacle) -> 100
            np.maximum.at(unified_grid, (gy_idx, gx_idx), vals)

        # 4. Construct and publish the unified OccupancyGrid
        now = self.get_clock().now()
        unified_msg = OccupancyGrid()
        unified_msg.header.stamp = now.to_msg()
        unified_msg.header.frame_id = self.merged_frame_id

        unified_msg.info.map_load_time = now.to_msg()
        unified_msg.info.resolution = float(res)
        unified_msg.info.width = int(w_glob)
        unified_msg.info.height = int(h_glob)
        unified_msg.info.origin.position.x = float(min_x)
        unified_msg.info.origin.position.y = float(min_y)
        unified_msg.info.origin.position.z = 0.0
        unified_msg.info.origin.orientation.w = 1.0

        unified_msg.data = unified_grid.ravel().tolist()

        self.merged_map_pub.publish(unified_msg)
        self.metadata_pub.publish(unified_msg.info)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
