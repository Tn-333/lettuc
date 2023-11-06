import sys

import rclpy
from rclpy.node import Node

from xarm_msgs.srv import SetDigitalIO
from std_msgs.msg import String




class AutoOpenClose(Node):
    def __init__(self):
        super().__init__("auto_open_close")

        self.cli = self.create_client(SetDigitalIO, "/xarm/set_tgpio_digital")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SetDigitalIO.Request()

        self.subscription = self.create_subscription(String, "auto_open_close", self.send_request, 10)
        self.subscription

    def send_request(self, effector_signal):
        self.req.ionum = 0

        if effector_signal.data == "open":
            self.req.value = 0  # 開く
        elif effector_signal == "close":
            self.req.value = 1  # 閉じる

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(argv=None):
    rclpy.init(args=argv)

    auto_open_close = AutoOpenClose()
    response = auto_open_close.send_request(True)  
    auto_open_close.get_logger().info("open/close")

    auto_open_close.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
