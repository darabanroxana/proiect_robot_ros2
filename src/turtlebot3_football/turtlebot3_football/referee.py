import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import os
import random
import math

# Servicii pentru pauza fizica
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from std_srvs.srv import Empty

class Referee(Node):
    def __init__(self):
        super().__init__('referee')
        
        self.ball_name = 'football_red_ball'
        self.goalie_name = 'goalie_block' 
        self.robot_name = None 
        
        self.score = 0
        self.cooldown_seconds = 3.0
        self.last_goal_timestamp = 0.0
        self.goal_phrases = ["Nice shot!", "Goal!", "Super goal!", "Victory!"]

        self.reset_pub = self.create_publisher(Bool, '/game_reset', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        self.pause_physics_client = self.create_client(Empty, '/gazebo/pause_physics')
        self.unpause_physics_client = self.create_client(Empty, '/gazebo/unpause_physics')

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            qos_profile
        )

        self.get_logger().info("ARBITRU: Conectat! Controlez portarul si verific OUT-urile.")
        
        # --- PAUZA DE INCALZIRE (10 SECUNDE) ---
        self.get_logger().info("⏳ Se asteapta 10 secunde pentru incarcarea stadionului...")
        time.sleep(10.0) # <--- Aici e pauza ceruta
        
        # --- INTRODUCERE AUDIO & FLUIER DE START ---
        self.get_logger().info("📢 Se anunta inceputul meciului...")
        os.system('espeak "Welcome to the Robot Football Champions League!" &')
        
        # Asteptam putin sa termine fraza
        time.sleep(3.5)
        
        # Numaratoare inversa si Fluier
        os.system('espeak "Three... Two... One... Start! Game On!" &')


    def model_states_callback(self, msg):
        # 1. Identificare Robot
        if self.robot_name is None:
            for name in msg.name:
                if ('turtle' in name.lower() or 'waffle' in name.lower()) and 'ball' not in name.lower():
                    self.robot_name = name
                    break

        if self.ball_name in msg.name:
            try:
                ball_idx = msg.name.index(self.ball_name)
                ball_pose = msg.pose[ball_idx]
                ball_x = ball_pose.position.x
                ball_y = ball_pose.position.y 
                
                # Miscare portar
                self.move_goalie_patrol()

                current_time = time.time()
                
                # Limita latimii portii
                GOAL_Y_LIMIT = 2.0

                # --- CAZ 1: CAPAT DREAPTA (POARTA OFICIALA) ---
                if ball_x > 4.6:
                    # A. GOL
                    if abs(ball_y) < GOAL_Y_LIMIT:
                        if (current_time - self.last_goal_timestamp) > self.cooldown_seconds:
                            self.score += 1
                            phrase = random.choice(self.goal_phrases)
                            
                            self.get_logger().warn(f"\n!!! {phrase.upper()} !!! SCOR OFICIAL: {self.score}")
                            os.system(f'espeak "{phrase}" &')
                            
                            self.reset_game_sequence()
                            self.last_goal_timestamp = current_time
                    # B. OUT
                    else:
                        if (current_time - self.last_goal_timestamp) > self.cooldown_seconds:
                            self.get_logger().info("\n... OUT! Mingea a trecut pe langa poarta.")
                            os.system('espeak "Missed shot." &')
                            self.reset_game_sequence()
                            self.last_goal_timestamp = current_time

                # --- CAZ 2: CAPAT STANGA (POARTA ANTRENAMENT) ---
                elif ball_x < -4.4:
                    # A. GOL ANTRENAMENT
                    if abs(ball_y) < GOAL_Y_LIMIT:
                        if (current_time - self.last_goal_timestamp) > self.cooldown_seconds:
                            self.get_logger().info(f"\n-> Gol de antrenament (Stanga). Robotul se intoarce!")
                            os.system('espeak "Training over." &')
                            
                            self.reset_game_sequence()
                            self.last_goal_timestamp = current_time
                    # B. OUT ANTRENAMENT
                    else:
                        if (current_time - self.last_goal_timestamp) > self.cooldown_seconds:
                            self.get_logger().info("\n... OUT la antrenament.")
                            os.system('espeak "Missed." &')
                            self.reset_game_sequence()
                            self.last_goal_timestamp = current_time

            except ValueError:
                pass

    def move_goalie_patrol(self):
        now = time.time()
        # Viteza adaptiva
        current_speed = min(1.5 + (self.score * 0.5), 5.0)
        amplitude = 1.8 
        final_y = amplitude * math.sin(now * current_speed)

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.goalie_name
        req.state.pose.position.x = 4.9
        req.state.pose.position.y = final_y
        req.state.pose.position.z = 0.2
        req.state.reference_frame = 'world'
        
        self.set_entity_state_client.call_async(req)

    def reset_game_sequence(self):
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        time.sleep(0.5)

        if self.pause_physics_client.service_is_ready():
          self.pause_physics_client.call_async(Empty.Request())
        
        # Reset MINGE la CENTRU
        offset_y = random.uniform(-0.3, 0.3)
        self.teleport_entity(self.ball_name, 0.0, offset_y, 0.2)
        
        # Reset Portar la centru
        self.teleport_entity(self.goalie_name, 4.9, 0.0, 0.2)

        time.sleep(0.2)

        if self.unpause_physics_client.service_is_ready():
          self.unpause_physics_client.call_async(Empty.Request())

        self.get_logger().info(f"-> Minge la centru. Ping-Pong continua!")

        turn_twist = Twist()
        turn_twist.angular.z = 1.5
        for _ in range(8):
            self.cmd_vel_pub.publish(turn_twist)
            time.sleep(0.1)

        self.cmd_vel_pub.publish(stop_twist)
        time.sleep(0.5)

        msg.data = False
        self.reset_pub.publish(msg)

    def teleport_entity(self, name, x, y, z):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self.set_entity_state_client.call_async(req)

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