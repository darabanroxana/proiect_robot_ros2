import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import os
import random
import math

GOALIE_SDF = """
<sdf version='1.6'>
  <model name='goalie_block'>
    <pose>0 0 0.2 0 0 0</pose>
    <link name='link'>
      <inertial>
        <mass>100</mass>
        <inertia>
          <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.2 0.6 0.4</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.2 0.6 0.4</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Blue</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

class Referee(Node):
    def __init__(self):
        super().__init__('referee')
        
        self.ball_name = 'football_red_ball'
        self.goalie_name = 'goalie_block'
        self.robot_name = None 
        
        self.score = 0
        # Pragul liniei portii (mai mic decat pozitia portarului pentru a evita coliziunea)
        self.goal_x_threshold = 4.6
        self.cooldown_seconds = 3.0
        self.last_goal_timestamp = 0.0
        self.goal_phrases = ["Nice shot!", "Goal!", "Super goal!", "Victory!"]

        self.reset_pub = self.create_publisher(Bool, '/game_reset', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            qos_profile
        )

        self.spawn_goalie()
        self.get_logger().info("ARBITRU: Portarul este gata de joc.")

    def spawn_goalie(self):
        req = SpawnEntity.Request()
        req.name = self.goalie_name
        req.xml = GOALIE_SDF
        # Pozitia portarului (mai in spate)
        req.initial_pose.position.x = 4.9 
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.2
        
        if self.spawn_entity_client.service_is_ready():
            self.spawn_entity_client.call_async(req)

    def model_states_callback(self, msg):
        # Identificare Robot
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
                
                # Portarul patruleaza continuu
                self.move_goalie_patrol()

                # Verificare Gol
                if ball_x > self.goal_x_threshold:
                    current_time = time.time()
                    if (current_time - self.last_goal_timestamp) > self.cooldown_seconds:
                        self.score += 1
                        phrase = random.choice(self.goal_phrases)
                        self.get_logger().warn(f"\n!!! {phrase.upper()} !!! Scor: {self.score}\n")
                        os.system(f'espeak "{phrase}" &')
                        
                        self.reset_game_sequence()
                        self.last_goal_timestamp = current_time
            except ValueError:
                pass

    def move_goalie_patrol(self):
        # --- MODIFICARE: VITEZA ADAPTIVA ---
        now = time.time()
        
        # Viteza de baza 1.5 + (0.5 * Scor)
        # La scor 0 -> viteza 1.5
        # La scor 2 -> viteza 2.5
        # Limitam la 5.0 ca sa nu o ia razna fizica
        current_speed = min(1.5 + (self.score * 0.5), 5.0)
        
        amplitude = 1.8 
        final_y = amplitude * math.sin(now * current_speed)

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.goalie_name
        req.state.pose.position.x = 4.8 
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

        # Reset Minge
        offset_y = random.uniform(-1.0, 1.0)
        self.teleport_entity(self.ball_name, 0.0, offset_y, 0.2)
        
        # Reset Portar la centru (temporar, apoi reia patrula)
        self.teleport_entity(self.goalie_name, 4.9, 0.0, 0.2)

        self.get_logger().info(f"-> GOL! Resetare completă.")

        # Rotire Robot (celebration)
        turn_twist = Twist()
        turn_twist.angular.z = 1.0
        for _ in range(15):
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