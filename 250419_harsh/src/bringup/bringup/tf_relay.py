import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')

        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # Publishers for robot1
        self.pub_r1_tf = self.create_publisher(TFMessage, '/robot1/tf', tf_qos)
        self.pub_r1_static = self.create_publisher(TFMessage, '/robot1/tf_static', static_qos)

        # Publishers for robot2
        self.pub_r2_tf = self.create_publisher(TFMessage, '/robot2/tf', tf_qos)
        self.pub_r2_static = self.create_publisher(TFMessage, '/robot2/tf_static', static_qos)

        # Subscriptions to global TF
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, tf_qos)
        self.sub_static = self.create_subscription(TFMessage, '/tf_static', self.static_callback, static_qos)

    def tf_callback(self, msg):
        self.pub_r1_tf.publish(msg)
        self.pub_r2_tf.publish(msg)

    def static_callback(self, msg):
        self.pub_r1_static.publish(msg)
        self.pub_r2_static.publish(msg)

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
