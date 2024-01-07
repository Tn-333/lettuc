import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from marker_detection_msg.msg import MarkerCoordinates


class OperationInst(Node):

    def __init__(self):
        super().__init__('operation_inst')
        self.subscription = self.create_subscription(
            MarkerCoordinates,
            'marker_coordinates_topic',
            self.listener_callback,
            10)
        
        self.feedback_subscription = self.create_subscription(Bool, "feedback_topic", self.recieve_feedback,10)
        
        self.publisher = self.create_publisher(String, "move_origin_topic", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.operate_arm)

        self.x = 0
        self.y = 0
        self.start_flag = False
        self.feedback = True

    def listener_callback(self, msgs):
        self.x = msgs.marker_x
        self.y = msgs.marker_y
        self.start_flag = True
    
    def recieve_feedback(self, flag):
        self.feedback = flag.data
    
    def operate_arm(self):
        msg = String()

        if not self.start_flag:
            return
        
        if not self.feedback:
            return
        
        self.get_logger().info("x = " + str(self.x))
        self.get_logger().info("y = " + str(self.y))

        if self.y < 240:
            action = "y_down"
        elif self.y > 250:
            action = "y_up"
        elif self.x < 340:
            action = "x_up"
        elif self.x > 350:
            action = "x_down"
        else:
            action = "set_complete"

        msg.data = action

        self.publisher.publish(msg)
        self.get_logger().info('publishing: "%s"' % msg.data)
        self.start_flag = False
        self.feedback = False



def main(args=None):
    rclpy.init(args=args)

    operation_inst = OperationInst()

    rclpy.spin(operation_inst)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    operation_inst.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()