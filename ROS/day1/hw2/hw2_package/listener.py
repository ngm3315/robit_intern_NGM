import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32

class MultiListener(Node):
    def __init__(self):
        super().__init__('multi_listener')
        self.create_subscription(Int32, 'int_topic', self.int_callback, 10)
        self.create_subscription(String, 'string_topic', self.string_callback, 10)
        self.create_subscription(Float32, 'float_topic', self.float_callback, 10)

    def int_callback(self, msg):
        self.get_logger().info(f'Recv int={msg.data}')

    def string_callback(self, msg):
        self.get_logger().info(f'Recv str={msg.data}')

    def float_callback(self, msg):
        self.get_logger().info(f'Recv float={msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
