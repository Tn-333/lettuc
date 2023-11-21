import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImgReceiver(Node):

    def __init__(self):
        super().__init__('img_receiver')

        self.br = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(
            Image,
            'processed', 10)

    def image_callback(self, data):
        self.get_logger().info('got image!')

        source = self.br.imgmsg_to_cv2(data, 'bgr8')
        edges = cv2.Canny(source, 50, 150)  # 50と150は下限と上限の閾値
        result_msg = self.br.cv2_to_imgmsg(edges, 'passthrough')
        self.publisher.publish(result_msg)
        self.get_logger().info('publish image!')

        pass


def main():
    rclpy.init()
    img_receiver = ImgReceiver()
    try:
        rclpy.spin(img_receiver)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

