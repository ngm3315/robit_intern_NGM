import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32

class MultiPublisher(Node):
    def __init__(self):
        super().__init__('multi_publisher')
        self.pub_int = self.create_publisher(Int32, 'int_topic', 10)
        self.pub_str = self.create_publisher(String, 'string_topic', 10)
        self.pub_flt = self.create_publisher(Float32, 'float_topic', 10)
        self.count = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg_int = Int32()
        msg_int.data = self.count

        msg_str = String()
        msg_str.data = f'Hello {self.count}'

        msg_flt = Float32()
        msg_flt.data = 3.14 * self.count

        self.pub_int.publish(msg_int)
        self.pub_str.publish(msg_str)
        self.pub_flt.publish(msg_flt)

        self.get_logger().info(
            f'Published -> int={msg_int.data}, str={msg_str.data}, float={msg_flt.data:.2f}'
        )

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MultiPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
