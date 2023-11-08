import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class OpneCloseTest(Node):

    def __init__(self):
        super().__init__('open_close_test')
        self.publisher_ = self.create_publisher(String, 'auto_open_close', 10)
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.i = 0

    def callback(self):
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

    open_close_test = OpneCloseTest()

    rclpy.spin(open_close_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    open_close_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()