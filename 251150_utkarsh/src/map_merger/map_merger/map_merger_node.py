#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_ros

class MapMergerNode(Node):
    def __init__(self):
        super().__init__('map_merger')

        # Declare parameters for robot initial spawn offsets
        self.declare_parameter('robot1_x', -1.5)
        self.declare_parameter('robot1_y', 0.5)
        self.declare_parameter('robot1_yaw', 0.0)

        self.declare_parameter('robot2_x', 1.5)
        self.declare_parameter('robot2_y', -0.5)
        self.declare_parameter('robot2_yaw', 0.0)

        self.r1_x = self.get_parameter('robot1_x').value
        self.r1_y = self.get_parameter('robot1_y').value
        self.r1_yaw = self.get_parameter('robot1_yaw').value

        self.r2_x = self.get_parameter('robot2_x').value
        self.r2_y = self.get_parameter('robot2_y').value
        self.r2_yaw = self.get_parameter('robot2_yaw').value

        self.get_logger().info('Map Merger Node Initialized')
        self.get_logger().info(f'Robot 1 spawn offset: ({self.r1_x:.2f}, {self.r1_y:.2f})')
        self.get_logger().info(f'Robot 2 spawn offset: ({self.r2_x:.2f}, {self.r2_y:.2f})')

        # TF Broadcaster for map -> robot1/map and map -> robot2/map
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # QoS for OccupancyGrid
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.map_r1 = None
        self.map_r2 = None

        self.sub_r1 = self.create_subscription(OccupancyGrid, '/robot1/map', self.map1_cb, map_qos)
        self.sub_r2 = self.create_subscription(OccupancyGrid, '/robot2/map', self.map2_cb, map_qos)

        self.pub_merged = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Timer for TF broadcast and map merging (2 Hz)
        self.timer = self.create_timer(0.5, self.loop)

    def map1_cb(self, msg):
        self.map_r1 = msg

    def map2_cb(self, msg):
        self.map_r2 = msg

    def broadcast_tf(self):
        now = self.get_clock().now().to_msg()

        # map -> robot1/map
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'robot1/map'
        t1.transform.translation.x = float(self.r1_x)
        t1.transform.translation.y = float(self.r1_y)
        t1.transform.translation.z = 0.0
        t1.transform.rotation.z = math.sin(self.r1_yaw / 2.0)
        t1.transform.rotation.w = math.cos(self.r1_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t1)

        # map -> robot2/map
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'robot2/map'
        t2.transform.translation.x = float(self.r2_x)
        t2.transform.translation.y = float(self.r2_y)
        t2.transform.translation.z = 0.0
        t2.transform.rotation.z = math.sin(self.r2_yaw / 2.0)
        t2.transform.rotation.w = math.cos(self.r2_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t2)

    def loop(self):
        self.broadcast_tf()

        # Merge if at least one map is available
        if self.map_r1 is None and self.map_r2 is None:
            return

        maps_to_merge = []
        if self.map_r1 is not None:
            maps_to_merge.append((self.map_r1, self.r1_x, self.r1_y))
        if self.map_r2 is not None:
            maps_to_merge.append((self.map_r2, self.r2_x, self.r2_y))

        # Calculate bounding box in global map coordinates
        res = maps_to_merge[0][0].info.resolution
        min_x = float('inf')
        min_y = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')

        for m, ox, oy in maps_to_merge:
            gx = m.info.origin.position.x + ox
            gy = m.info.origin.position.y + oy
            gw = m.info.width * res
            gh = m.info.height * res
            min_x = min(min_x, gx)
            min_y = min(min_y, gy)
            max_x = max(max_x, gx + gw)
            max_y = max(max_y, gy + gh)

        merged_w = int(np.ceil((max_x - min_x) / res))
        merged_h = int(np.ceil((max_y - min_y) / res))

        if merged_w <= 0 or merged_h <= 0:
            return

        # Initialize with -1 (unknown)
        merged_grid = np.full((merged_h, merged_w), -1, dtype=np.int8)

        for m, ox, oy in maps_to_merge:
            w, h = m.info.width, m.info.height
            data = np.array(m.data, dtype=np.int8).reshape((h, w))
            gx = m.info.origin.position.x + ox
            gy = m.info.origin.position.y + oy

            ix0 = int(round((gx - min_x) / res))
            iy0 = int(round((gy - min_y) / res))

            sub = merged_grid[iy0:iy0+h, ix0:ix0+w]

            # Obstacles (>= 50) have highest priority
            occ = (data >= 50)
            # Free (0) overrides unknown (-1), but not occupied
            free = (data == 0) & (sub < 50)

            sub[free] = 0
            sub[occ] = 100

        # Construct OccupancyGrid message
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'map'

        merged_msg.info.resolution = res
        merged_msg.info.width = merged_w
        merged_msg.info.height = merged_h
        merged_msg.info.origin.position.x = min_x
        merged_msg.info.origin.position.y = min_y
        merged_msg.info.origin.position.z = 0.0
        merged_msg.info.origin.orientation.w = 1.0

        merged_msg.data = merged_grid.flatten().tolist()
        self.pub_merged.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
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
