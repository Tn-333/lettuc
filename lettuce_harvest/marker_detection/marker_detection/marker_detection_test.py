import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

from marker_detection_msg.msg import MarkerCoordinates
from std_msgs.msg import Bool

class ImgReceiver(Node):

    def __init__(self):
        super().__init__('img_receiver')
        time.sleep(3)  # 移動時間考慮

        self.br = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(
            Image,
            'processed', 10)

        self.marker_coord_publisher = self.create_publisher(
            MarkerCoordinates,
            "marker_coordinates_topic", 10
        )

        self.marker_found_publisher = self.create_publisher(Bool, "marker_found_topic", 10)

        self.x = 0
        self.y = 0
        self.marker_found = False

    def coords_callback(self):
        msgs = MarkerCoordinates()

        msgs.marker_x = int(self.x)
        msgs.marker_y = int(self.y)

        self.marker_coord_publisher.publish(msgs)

    def image_callback(self, data):
        # self.get_logger().info('画像を取得しました！')

        source = self.br.imgmsg_to_cv2(data, 'bgr8')

        # 画像をHSVカラースペースに変換
        hsv = cv2.cvtColor(source, cv2.COLOR_BGR2HSV)

        # 赤い色のHSV範囲を定義
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # ピンク色のHSV範囲を定義
        lower_pink = np.array([140, 100, 100])
        upper_pink = np.array([170, 255, 255])

        # 赤い色とピンク色のバイナリマスクを作成
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)

        # マスクを合成してピンク色も検出
        mask = cv2.bitwise_or(mask_red, mask_pink)

        # マスク内の輪郭を検出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 輪郭が存在する場合
        if contours:
            # 最大の輪郭を取得（一番大きな円を仮定）
            max_contour = max(contours, key=cv2.contourArea)

            # 輪郭を外接する円を検出
            (x, y), radius = cv2.minEnclosingCircle(max_contour)

            # 中心座標を整数に変換
            center = (int(x), int(y))

            # 半径を整数に変換
            radius = int(radius)

            # 中心座標を描画
            cv2.circle(source, center, radius, (0, 255, 0), 2)
            cv2.circle(source, center, 5, (0, 0, 255), -1)

            self.x = x
            self.y = y
            self.marker_found = True

            # 中心座標をログに出力
            # self.get_logger().info(f'円の中心座標: {center}')

            self.coords_callback()

        self.marker_msg_callback()  # アーム動作にboolを飛ばす
        result_msg = self.br.cv2_to_imgmsg(source, 'bgr8')
        self.publisher.publish(result_msg)
        # self.get_logger().info('画像を公開しました！')

    def marker_msg_callback(self):
        msg = Bool()
        msg.data = self.marker_found
        self.marker_found_publisher.publish(msg)

        print("x = " + str(self.x))
        print("y = " + str(self.y))
        # self.get_logger().info("marker was found! harvest is biginning")

def main():
    rclpy.init()
    img_receiver = ImgReceiver()
    try:
        rclpy.spin(img_receiver)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
