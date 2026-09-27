#!/usr/bin/env python3
"""
Real-Time Map Merger Node for Multi-Robot System in ROS 2 Humble.
Fuses OccupancyGrid maps from robot1 and robot2 into a unified global map.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class MapMergerNode(Node):
    """Fuses multi-robot OccupancyGrid maps into a unified global /map."""

    def __init__(self):
        super().__init__('map_merger_node')

        # Parameters
        self.declare_parameter('robot1_map_topic', '/robot1/map')
        self.declare_parameter('robot2_map_topic', '/robot2/map')
        self.declare_parameter('merged_map_topic', '/map')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('publish_rate', 1.0)

        r1_topic = self.get_parameter('robot1_map_topic').get_parameter_value().string_value
        r2_topic = self.get_parameter('robot2_map_topic').get_parameter_value().string_value
        merged_topic = self.get_parameter('merged_map_topic').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.get_logger().info(
            f'MapMerger initialized: r1={r1_topic}, r2={r2_topic} -> merged={merged_topic} (frame={self.output_frame})'
        )

        # QoS Profiles (Transient Local for latched occupancy grids)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.map1_sub = self.create_subscription(OccupancyGrid, r1_topic, self.map1_cb, map_qos)
        self.map2_sub = self.create_subscription(OccupancyGrid, r2_topic, self.map2_cb, map_qos)

        # Publisher
        self.merged_pub = self.create_publisher(OccupancyGrid, merged_topic, map_qos)

        # Static transform broadcaster to link 'map' to 'robot1/map' and 'robot2/map'
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tfs()

        self.map1 = None
        self.map2 = None

        # Merge timer
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.merge_and_publish)

    def publish_static_tfs(self):
        """Link unified 'map' frame to each robot's map frame."""
        now = self.get_clock().now().to_msg()
        tfs = []

        for r_name in ['robot1', 'robot2']:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.output_frame
            t.child_frame_id = f'{r_name}/map'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            tfs.append(t)

        self.static_tf_broadcaster.sendTransform(tfs)

    def map1_cb(self, msg: OccupancyGrid):
        self.map1 = msg

    def map2_cb(self, msg: OccupancyGrid):
        self.map2 = msg

    def merge_and_publish(self):
        """Perform vectorized bounding-box grid fusion and publish merged map."""
        if self.map1 is None and self.map2 is None:
            return

        # Case 1: Only map1 is available
        if self.map1 is not None and self.map2 is None:
            out_msg = OccupancyGrid()
            out_msg.header = self.map1.header
            out_msg.header.frame_id = self.output_frame
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.info = self.map1.info
            out_msg.data = self.map1.data
            self.merged_pub.publish(out_msg)
            return

        # Case 2: Only map2 is available
        if self.map2 is not None and self.map1 is None:
            out_msg = OccupancyGrid()
            out_msg.header = self.map2.header
            out_msg.header.frame_id = self.output_frame
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.info = self.map2.info
            out_msg.data = self.map2.data
            self.merged_pub.publish(out_msg)
            return

        # Case 3: Both maps available -> Perform fusion
        m1 = self.map1
        m2 = self.map2

        res = min(m1.info.resolution, m2.info.resolution)
        if res <= 0:
            return

        ox1 = m1.info.origin.position.x
        oy1 = m1.info.origin.position.y
        ox2 = m2.info.origin.position.x
        oy2 = m2.info.origin.position.y

        w1, h1 = m1.info.width, m1.info.height
        w2, h2 = m2.info.width, m2.info.height

        if w1 <= 0 or h1 <= 0 or w2 <= 0 or h2 <= 0:
            return

        # Global bounding box in world meters
        min_x = min(ox1, ox2)
        min_y = min(oy1, oy2)
        max_x = max(ox1 + w1 * m1.info.resolution, ox2 + w2 * m2.info.resolution)
        max_y = max(oy1 + h1 * m1.info.resolution, oy2 + h2 * m2.info.resolution)

        merged_w = int(math.ceil((max_x - min_x) / res))
        merged_h = int(math.ceil((max_y - min_y) / res))

        if merged_w <= 0 or merged_h <= 0 or merged_w > 5000 or merged_h > 5000:
            return

        # 2D numpy arrays
        grid1 = np.array(m1.data, dtype=np.int8).reshape((h1, w1))
        grid2 = np.array(m2.data, dtype=np.int8).reshape((h2, w2))

        # Blank canvas for each robot
        c1 = np.full((merged_h, merged_w), -1, dtype=np.int8)
        c2 = np.full((merged_h, merged_w), -1, dtype=np.int8)

        # Offsets
        x1_s = int(round((ox1 - min_x) / res))
        y1_s = int(round((oy1 - min_y) / res))
        x2_s = int(round((ox2 - min_x) / res))
        y2_s = int(round((oy2 - min_y) / res))

        # Bounds clipping to prevent edge overflow
        x1_e = min(x1_s + w1, merged_w)
        y1_e = min(y1_s + h1, merged_h)
        x2_e = min(x2_s + w2, merged_w)
        y2_e = min(y2_s + h2, merged_h)

        c1[y1_s:y1_e, x1_s:x1_e] = grid1[:y1_e - y1_s, :x1_e - x1_s]
        c2[y2_s:y2_e, x2_s:x2_e] = grid2[:y2_e - y2_s, :x2_e - x2_s]

        # Vectorized fusion:
        # 1. Unknown + Known -> Known
        # 2. Both known -> Maximum (preserves obstacles over free space)
        merged = np.full((merged_h, merged_w), -1, dtype=np.int8)
        mask1 = (c1 >= 0) & (c2 < 0)
        mask2 = (c2 >= 0) & (c1 < 0)
        both = (c1 >= 0) & (c2 >= 0)

        merged[mask1] = c1[mask1]
        merged[mask2] = c2[mask2]
        merged[both] = np.maximum(c1[both], c2[both])

        # Publish merged OccupancyGrid
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = self.output_frame
        merged_msg.info.resolution = float(res)
        merged_msg.info.width = int(merged_w)
        merged_msg.info.height = int(merged_h)
        merged_msg.info.origin.position.x = float(min_x)
        merged_msg.info.origin.position.y = float(min_y)
        merged_msg.info.origin.position.z = 0.0
        merged_msg.info.origin.orientation.w = 1.0
        merged_msg.data = merged.ravel().tolist()

        self.merged_pub.publish(merged_msg)


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
