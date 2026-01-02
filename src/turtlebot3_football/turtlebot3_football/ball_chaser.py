import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()


        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                width = cv_image.shape[1]
                center_left = width // 3
                center_right = 2 * width // 3

                if center_left < cx < center_right:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
                elif cx <= center_left:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # Search mode: spin in place to find the ball
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()