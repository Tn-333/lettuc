import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool

import time


class DemoPublish(Node):

    def __init__(self):
        super().__init__('demo_publish')
        self.publisher_ = self.create_publisher(String, 'move_origin_topic', 10)
        self.subscription = self.create_subscription(Bool, "feedback_topic", self.recieve_flag, 10)
        self.flag = True

        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.i = 1

    def recieve_flag(self, msg):
        print("Start processing")
        self.flag = msg.data

    def callback(self):
        msg = String()

        if self.i == 1:
            msg.data = "y_up"
        elif self.i == 2:
            msg.data = "x_up"
        elif self.i ==3:
            msg.data = "y_down"
        elif self.i == 4:
            msg.data = "x_down"
        elif self.i == 5:
            msg.data = "set_complete"

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        self.flag = False



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
