import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        
        self.bridge = CvBridge()
        
        # --- CONFIGURĂRI ---
        self.max_speed = 0.7        
        self.search_speed = 0.5    
        self.turn_sensitivity = 0.01
        
        # LIMITA DE FRÂNARE (3.2m = Foarte sigur, departe de portar)
        self.safety_limit_x = 3.2
        
        self.robot_name = None 
        self.robot_x = -999.0 
        self.game_active = True 

        # Publisheri
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriberi
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.reset_sub = self.create_subscription(Bool, '/game_reset', self.reset_callback, 10)
        
        # Subscriber Poziție
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pose_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.pose_callback, qos)
        
        # Timer pentru a afișa poziția (Diagnostic)
        self.create_timer(0.5, self.print_status)
        
        self.get_logger().info(f"✅ Jucator ACTIV. Frana setata la {self.safety_limit_x}m.")

    def reset_callback(self, msg):
        if msg.data: 
            self.game_active = False
            self.stop_robot()
        else:
            self.game_active = True

    def pose_callback(self, msg):
        # 1. Identificare automată a numelui
        if self.robot_name is None:
            for name in msg.name:
                # Căutăm exact waffle_pi pentru a evita confuziile
                if 'turtlebot3_waffle_pi' in name:
                    self.robot_name = name
                    self.get_logger().info(f"✅ Robot conectat: {self.robot_name}")
                    break
        
        # 2. Actualizare poziție
        if self.robot_name and self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            self.robot_x = msg.pose[idx].position.x

    def print_status(self):
        # Această funcție îți arată în terminal unde crede robotul că e
        if self.robot_x != -999.0:
            # Afișăm doar dacă robotul se mișcă spre poartă (> 0.5m)
            if self.robot_x > 0.5:
                print(f"📍 Pozitie curenta: {self.robot_x:.2f}m (Limita: {self.safety_limit_x}m)")

    def stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)

    def brake_active(self):
        # Frână puternică (spate)
        brake = Twist()
        brake.linear.x = -0.3
        brake.angular.z = 0.0
        self.cmd_pub.publish(brake)

    def image_callback(self, msg):
        if not self.game_active:
            self.stop_robot()
            return

        # --- LOGICA DE FRÂNARE ---
        if self.robot_x > self.safety_limit_x:
            self.get_logger().warn(f"🛑 LIMITA ATINSA ({self.robot_x:.2f}m) -> FRANARE FORTATA!", throttle_duration_sec=1.0)
            self.brake_active()
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        twist = Twist()

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                width = cv_image.shape[1]
                err_x = cx - width / 2
                
                twist.linear.x = self.max_speed
                twist.angular.z = -float(err_x) * self.turn_sensitivity
        else:
            twist.linear.x = 0.0
            twist.angular.z = self.search_speed 

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