import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class MapMergerNode(Node):
    def __init__(self):
        super().__init__('map_merger_node')

        self.get_logger().info('Map Merger Node started. Aligning local maps...')

        # Static TF broadcaster to link global "map" frame to both local map frames at origin
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_map_transforms()

        # Subscribers for both local occupancy grids
        self.map1 = None
        self.map2 = None

        self.sub_map1 = self.create_subscription(OccupancyGrid, '/robot1/map', self.map1_callback, 10)
        self.sub_map2 = self.create_subscription(OccupancyGrid, '/robot2/map', self.map2_callback, 10)

        # Publisher for the fused global map (Transient Local QoS for RViz)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_merged_map = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Merge timer: fuses maps every 1.0 second
        self.timer = self.create_timer(1.0, self.merge_and_publish)

    def broadcast_map_transforms(self):
        # Both robots' local map frames align with the global map frame origin
        transforms = []

        # map -> robot1/map
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'robot1/map'
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        # map -> robot2/map
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'robot2/map'
        t2.transform.rotation.w = 1.0
        transforms.append(t2)

        self.tf_broadcaster.sendTransform(transforms)

    def map1_callback(self, msg: OccupancyGrid):
        self.map1 = msg

    def map2_callback(self, msg: OccupancyGrid):
        self.map2 = msg

    def merge_and_publish(self):
        if self.map1 is None and self.map2 is None:
            return

        if self.map1 is not None and self.map2 is None:
            self.map1.header.frame_id = 'map'
            self.pub_merged_map.publish(self.map1)
            return
        elif self.map2 is not None and self.map1 is None:
            self.map2.header.frame_id = 'map'
            self.pub_merged_map.publish(self.map2)
            return

        # Both maps available: compute unified bounding box
        res = min(self.map1.info.resolution, self.map2.info.resolution)

        m1_ox = self.map1.info.origin.position.x
        m1_oy = self.map1.info.origin.position.y
        m1_w = self.map1.info.width
        m1_h = self.map1.info.height

        m2_ox = self.map2.info.origin.position.x
        m2_oy = self.map2.info.origin.position.y
        m2_w = self.map2.info.width
        m2_h = self.map2.info.height

        # Global bounding box encompassing both maps
        g_min_x = min(m1_ox, m2_ox)
        g_min_y = min(m1_oy, m2_oy)
        g_max_x = max(m1_ox + m1_w * res, m2_ox + m2_w * res)
        g_max_y = max(m1_oy + m1_h * res, m2_oy + m2_h * res)

        g_width = int(math.ceil((g_max_x - g_min_x) / res))
        g_height = int(math.ceil((g_max_y - g_min_y) / res))

        if g_width <= 0 or g_height <= 0:
            return

        # Initialize global grid with unknown (-1)
        merged = np.full((g_height, g_width), -1, dtype=np.int8)

        # Overlay a map directly into the global grid
        def overlay_map(occ_msg):
            w = occ_msg.info.width
            h = occ_msg.info.height
            ox = occ_msg.info.origin.position.x
            oy = occ_msg.info.origin.position.y
            data = np.array(occ_msg.data, dtype=np.int8).reshape((h, w))

            # Find cell index offsets in global grid
            x_offset = int(round((ox - g_min_x) / res))
            y_offset = int(round((oy - g_min_y) / res))

            # Place cells: take maximum value so obstacles (100) and free space (0) overwrite unknown (-1)
            target_slice = merged[y_offset:y_offset + h, x_offset:x_offset + w]
            merged[y_offset:y_offset + h, x_offset:x_offset + w] = np.maximum(target_slice, data)

        overlay_map(self.map1)
        overlay_map(self.map2)

        # Create output OccupancyGrid message
        out_msg = OccupancyGrid()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'map'
        out_msg.info.resolution = res
        out_msg.info.width = g_width
        out_msg.info.height = g_height
        out_msg.info.origin.position.x = g_min_x
        out_msg.info.origin.position.y = g_min_y
        out_msg.info.origin.position.z = 0.0
        out_msg.info.origin.orientation.w = 1.0
        out_msg.data = merged.flatten().tolist()

        self.pub_merged_map.publish(out_msg)


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
