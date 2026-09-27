#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def latched_qos(depth=1):
    """Transient-local: late joiners (RViz, Nav2's static layer) get the map."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def map_input_qos(depth=1):
    
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class MapMerger(Node):

    def __init__(self):
        super().__init__('map_merger')

        self.declare_parameter('robot_namespaces', ['tb3_0', 'tb3_1'])
        self.declare_parameter('merged_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('margin_cells', 4)

        self.namespaces = list(self.get_parameter('robot_namespaces').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.margin = int(self.get_parameter('margin_cells').value)

        self.maps = {ns: None for ns in self.namespaces}
        self.dirty = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for ns in self.namespaces:
            self.create_subscription(
                OccupancyGrid, f'/{ns}/map',
                lambda msg, n=ns: self._on_map(n, msg), map_input_qos())

        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('merged_topic').value, latched_qos())
        self.coverage_pub = self.create_publisher(Float32, 'map_merge/explored_area', 10)

        self.create_timer(float(self.get_parameter('publish_period').value), self._merge)
        self._log_counter = 0
        self._published_once = False

        self.get_logger().info(
            f'map_merger ready: merging {self.namespaces} into '
            f'"{self.get_parameter("merged_topic").value}" (frame "{self.global_frame}")')

    def _on_map(self, ns, msg):
        self.maps[ns] = msg
        self.dirty = True

    def _lookup(self, ns, frame):
        """Pose of `frame` expressed in the global frame, or None."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.global_frame, frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(
                f'{ns}: no transform {self.global_frame} <- {frame}: {exc}',
                throttle_duration_sec=5.0)
            return None
        t = tr.transform.translation
        return (t.x, t.y, yaw_from_quat(tr.transform.rotation))

    @staticmethod
    def _cells_in_global_frame(grid, pose):
        """Return (world_x, world_y, value) of every KNOWN cell of one map."""
        info = grid.info
        data = np.asarray(grid.data, dtype=np.int8).reshape(info.height, info.width)
        rows, cols = np.nonzero(data >= 0)
        if rows.size == 0:
            return None
        vals = data[rows, cols]

        res = info.resolution
        lx = (cols.astype(np.float64) + 0.5) * res
        ly = (rows.astype(np.float64) + 0.5) * res

        oyaw = yaw_from_quat(info.origin.orientation)
        co, so = math.cos(oyaw), math.sin(oyaw)
        mx = co * lx - so * ly + info.origin.position.x
        my = so * lx + co * ly + info.origin.position.y

        X, Y, T = pose
        ct, st = math.cos(T), math.sin(T)
        wx = ct * mx - st * my + X
        wy = st * mx + ct * my + Y
        return wx, wy, vals
    def _merge(self):
        if not self.dirty:
            return

        pending = []
        for ns in self.namespaces:
            grid = self.maps[ns]
            if grid is None:
                continue
            pose = self._lookup(ns, grid.header.frame_id)
            if pose is None:
                return
            pending.append((ns, grid, pose))

        chunks, res = [], None
        for ns, grid, pose in pending:
            if res is None:
                res = grid.info.resolution
            elif abs(res - grid.info.resolution) > 1e-6:
                self.get_logger().warn(
                    f'{ns} publishes resolution {grid.info.resolution} != {res}; '
                    'using the first one (set the same value in slam params)')
            out = self._cells_in_global_frame(grid, pose)
            if out is not None:
                chunks.append(out)

        if not chunks or res is None:
            return
        self.dirty = False

        min_x = min(c[0].min() for c in chunks)
        max_x = max(c[0].max() for c in chunks)
        min_y = min(c[1].min() for c in chunks)
        max_y = max(c[1].max() for c in chunks)

        gx0 = math.floor(min_x / res) * res - self.margin * res
        gy0 = math.floor(min_y / res) * res - self.margin * res
        width = int(math.ceil((max_x - gx0) / res)) + 1 + self.margin
        height = int(math.ceil((max_y - gy0) / res)) + 1 + self.margin

        merged = np.full((height, width), -1, dtype=np.int8)
        for wx, wy, vals in chunks:
            cols = ((wx - gx0) / res).astype(np.int64)
            rows = ((wy - gy0) / res).astype(np.int64)
            np.clip(cols, 0, width - 1, out=cols)
            np.clip(rows, 0, height - 1, out=rows)
            np.maximum.at(merged, (rows, cols), vals)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = gx0
        msg.info.origin.position.y = gy0
        msg.info.origin.orientation.w = 1.0
        msg.data = merged.reshape(-1).tolist()
        self.pub.publish(msg)

        known = int(np.count_nonzero(merged >= 0))
        area = Float32()
        area.data = float(known * res * res)
        self.coverage_pub.publish(area)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                f'first merged map published: {width}x{height} @ {res:.3f} m '
                f'from {len(chunks)} robot map(s)')

        self._log_counter += 1
        if self._log_counter % 15 == 0:
            self.get_logger().info(
                f'merged map {width}x{height} @ {res:.3f} m | '
                f'explored {area.data:.1f} m^2 | '
                f'occupied {int(np.count_nonzero(merged >= 65))} cells')


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
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
