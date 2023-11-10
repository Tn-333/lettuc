import rclpy
from rclpy.node import Node

from origin_coordinate_msgs.msg import OriginCoordinates
import time

class PointPubDemo(Node):
    def __init__(self):
        super().__init__('point_pub_demo')
        self.publisher = self.create_publisher(OriginCoordinates, 'origin_points_topic', 10)
        self.callback()


    def callback(self):
        msg = OriginCoordinates()

        msg.x = 200.0
        msg.y = 200.0
        msg.z = 100.0

        print("waiting 3s...")
        time.sleep(3)

        self.publisher.publish(msg)
        self.get_logger().info('x: "%s"' % msg.x)
        self.get_logger().info('y: "%s"' % msg.y)


def main(args=None):
    rclpy.init(args=args)

    point_pub_demo = PointPubDemo()

    rclpy.spin(point_pub_demo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_pub_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()