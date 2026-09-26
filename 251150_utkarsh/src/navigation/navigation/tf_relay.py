#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy

class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')

        # Publishers for robot namespaces
        self.pub1_tf = self.create_publisher(TFMessage, '/robot1/tf', 100)
        self.pub2_tf = self.create_publisher(TFMessage, '/robot2/tf', 100)

        qos = QoSProfile(depth=100)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub1_static = self.create_publisher(TFMessage, '/robot1/tf_static', qos)
        self.pub2_static = self.create_publisher(TFMessage, '/robot2/tf_static', qos)

        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_cb, 100)
        self.sub_static = self.create_subscription(TFMessage, '/tf_static', self.static_cb, qos)

        self.get_logger().info('TF Relay running: mirroring /tf and /tf_static to /robot1 and /robot2')

    def tf_cb(self, msg):
        self.pub1_tf.publish(msg)
        self.pub2_tf.publish(msg)

    def static_cb(self, msg):
        self.pub1_static.publish(msg)
        self.pub2_static.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
