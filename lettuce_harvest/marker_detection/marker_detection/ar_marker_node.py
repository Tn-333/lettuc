import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(msg):
    # Convert ROS Image message to OpenCV format
    cv_image = CvBridge().imgmsg_to_cv2(msg)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

    if ids is not None:
        # Draw markers and display center coordinates
        for i in range(len(ids)):
            c = corners[i][0]
            centroid = (int(np.mean(c[:, 0])), int(np.mean(c[:, 1])))
            
            # Draw marker outline
            cv2.polylines(cv_image, [c.astype(int)], True, (0, 255, 0), 2)
            
            # Draw marker center
            cv2.circle(cv_image, centroid, 5, (0, 255, 0), -1)
            
            # Display marker ID and center coordinates in ROS 2 log
            rclpy.get_logger().info(f"Marker ID: {ids[i][0]}, Center Coordinates: {centroid}")

        # Display the modified image
        cv2.imshow("Detected Markers", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = rclpy.create_node('aruco_node')
    node.create_subscription(Image, 'image_raw', callback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
