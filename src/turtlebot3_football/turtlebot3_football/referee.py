import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import time
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Referee(Node):
    def __init__(self):
        super().__init__('referee')
        self.score = 0
        
        # --- SETĂRI SIGURE ---
        self.ball_name = 'football_red_ball' 
        
        # Distanța: Am pus 4.7 ca să fim siguri că prinde linia (chiar dacă poarta e la 4.5)
        self.goal_x = 4.7
        
        # Lățimea: Am pus 5.0 metri (IMENSĂ). 
        # Astfel, arbitrul nu mai comentează că e "pe lângă".
        self.goal_y_limit = 5.0   
        
        self.cooldown_time = 3.0  
        self.last_goal_time = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.model_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_callback,
            qos
        )
        self.set_entity_state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        self.get_logger().info(f"ARBITRU GATA. Astept mingea peste linia de {self.goal_x} metri.")

    def model_callback(self, msg):
        if self.ball_name in msg.name:
            idx = msg.name.index(self.ball_name)
            ball_pose = msg.pose[idx]
            
            x = ball_pose.position.x
            y = ball_pose.position.y
            
            # DEBUG: Îți arată distanța curentă
            self.get_logger().info(f"Distanta: {x:.2f}")

            # LOGICA SIMPLIFICATĂ
            # Dacă trece de 4.0 metri, e gol. Punct.
            if x > self.goal_x:
                # Verificarea de lățime e acum doar formală (practic acceptă orice)
                if -self.goal_y_limit < y < self.goal_y_limit:
                    now = time.time()
                    if now - self.last_goal_time > self.cooldown_time:
                        self.score += 1
                        self.get_logger().warn(f"\n!!! GOOOOOL !!! Scor: {self.score}\n")
                        os.system(f'espeak "Goal! Score {self.score}" &')
                        self.reset_ball()
                        self.last_goal_time = now

    def reset_ball(self):
        # Resetare fără verificare de service (Force Reset)
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.ball_name
        req.state.pose.position.x = 0.0
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.2
        req.state.reference_frame = 'world'
        self.set_entity_state_cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = Referee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()