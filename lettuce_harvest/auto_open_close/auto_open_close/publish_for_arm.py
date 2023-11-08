import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DemoPublish(Node):

    def __init__(self):
        super().__init__('demo_publish')
        self.publisher_ = self.create_publisher(String, 'move_origin_topic', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.i = 1

    def callback(self):
        msg = String()
        if self.i % 4 == 0:
            msg.data = "y_down"
        elif self.i % 3 == 0:
            msg.data = "y_up"
        elif self.i % 2 == 0:
            msg.data = "x_down"
        else:
            msg.data = "x_up"

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    demo_publish = DemoPublish()

    rclpy.spin(demo_publish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    demo_publish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()