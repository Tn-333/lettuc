from sys import flags
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
import time

class ImgReceiver(Node):

    def __init__(self):
        super().__init__('img_receiver')

        time.sleep(5) ##初期位置への移動時間の確保

        self.x_for_determination = 0
        self.y_for_determination = 0
        self.flag = True
        self.marker_found = False
        self.feedback = True
        self.count = 0

        self.br = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        
        self.feedback_subscription = self.create_subscription(Bool, "feedback_topic", self.recieve_feedback, 10)

        self.publisher = self.create_publisher(
            Image,
            'processed', 10)
        
        self.marker_found_publisher = self.create_publisher(Bool, "marker_found_topic", 10)
        #timer_period = 5.0  # seconds
        #self.timer = self.create_timer(timer_period, self.callback)

        #self.command_publisher = self.create_publisher(String, "move_origin_topic", 10)
        
        
        self.instruction_publisher = self.create_publisher(String, "move_origin_topic", 10)

        ##timer_period = 0.1
        ##self.timer = self.create_timer(timer_period, self.marker_msg_callback)

        #timer_period_commamd = 3.0
        #self.create_timer(timer_period_commamd, self.command_movement)



    def image_callback(self, data):
        ##self.get_logger().info('画像を取得しました！')

        source = self.br.imgmsg_to_cv2(data, 'bgr8')

        # 画像をHSVカラースペースに変換
        hsv = cv2.cvtColor(source, cv2.COLOR_BGR2HSV)

        # 赤い色のHSV範囲を定義
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # 赤い色のバイナリマスクを作成
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # マスク内の輪郭を検出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 輪郭が存在する場合
        if contours:
            # 最大の輪郭を取得（一番大きな赤い円を仮定）
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

            # 中心座標をログに出力
            self.get_logger().info(f'赤い円の中心座標: {center}')
            #print(str(x) + " and " + str(y))

            self.x_for_determination = x
            self.y_for_determination = y

            #self.command_movement()
            #self.marker_found = True


        result_msg = self.br.cv2_to_imgmsg(source, 'bgr8')
        self.publisher.publish(result_msg)
        ##self.get_logger().info('画像を公開しました！')

    def command_movement(self):
        if not self.marker_found:
            return
        
        if not self.feedback:
            return

        msg = String()
        ##if not self.flag:
            ##return
        

        if self.y_for_determination < 130:
            msg.data = "x_up"
        elif self.y_for_determination > 180:
            msg.data = "x_down"
        elif self.x_for_determination < 300:
            msg.data = "y_up"
        elif self.x_for_determination > 400:
            msg.data = "y_down"
        else:
            msg.data = "set_complete"

        self.instruction_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.flag = False
        #self.marker_found = False

    def recieve_feedback(self, msg):
        self.get_logger().info("recieved feedback")
        self.feedback = msg.data


    def marker_msg_callback(self):
        msg = Bool()
        msg.data = self.marker_found
        self.marker_found_publisher.publish(msg)
        if msg.data:
            print("True")
        else:
            print("False")
        #self.get_logger().info("marker was found! harvest is biginning")
        self.feedback = False







def main():
    rclpy.init()
    img_receiver = ImgReceiver()
    try:
        rclpy.spin(img_receiver)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

