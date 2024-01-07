import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from marker_detection_msg.msg import MarkerCoordinates
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import time

class QrCodeReader(Node):

    def __init__(self):
        super().__init__('qr_code_reader')
        time.sleep(3)

        self.br = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        
        self.publisher = self.create_publisher(
            Image,
            "processed", 10)
        
        self.marker_coord_publisher = self.create_publisher(
            MarkerCoordinates,
            "marker_coordinates_topic", 10
        )

        self.marker_found_publisher = self.create_publisher(Bool, "marker_found_topic", 10)

        self.x = None
        self.y = None
        self.marker_found = False

    def coords_callback(self):
        msgs = MarkerCoordinates()

        msgs.marker_x = int(self.x)
        msgs.marker_y = int(self.y)
        self.marker_coord_publisher.publish(msgs)

    def image_callback(self, data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

        # グレースケールに変換
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # QRコードを検出
        decoded_objects = decode(gray)

        for obj in decoded_objects:
            points = obj.polygon
            if len(points) == 4:
                pts = [(point.x, point.y) for point in points]
                pts = cv2.convexHull(np.array(pts, dtype=int))
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)

                # 中心座標を計算
                center_x = int((pts[0][0][0] + pts[2][0][0]) / 2)
                center_y = int((pts[0][0][1] + pts[2][0][1]) / 2)

                self.x = center_x
                self.y = center_y
                self.marker_found = True

                # 中心座標を画像に描画
                cv2.putText(cv_image, f'Center: ({center_x}, {center_y})', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
                self.coords_callback()

        self.marker_msg_callback()

        #if (not (self.x is None)) and (not (self.y is None)):
            #self.coords_callback()

        # 処理結果を表示
        result_msg = self.br.cv2_to_imgmsg(cv_image, "bgr8")
        self.publisher.publish(result_msg)
        #cv2.imshow("QR Code Detection", cv_image)
        #cv2.waitKey(1)

    def marker_msg_callback(self):
        msg = Bool()
        msg.data = self.marker_found
        self.marker_found_publisher.publish(msg)

        #print("x = " + str(self.x))
        #print("y = " + str(self.y))

def main():
    rclpy.init()
    qr_code_reader = QrCodeReader()
    try:
        rclpy.spin(qr_code_reader)
    except KeyboardInterrupt:
        pass

    qr_code_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
