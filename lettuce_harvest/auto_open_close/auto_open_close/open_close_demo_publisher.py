import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DemoPublisher(Node):

    def __init__(self):
        super().__init__('demo_publisher')
        self.publisher_ = self.create_publisher(String, 'auto_open_close_msg', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        if self.i %2 == 0:
            msg.data = "open"
        else:
            msg.data = "close"

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    demo_publisher = DemoPublisher()

    rclpy.spin(demo_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    demo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()