import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose as TurtlePose
from nav_msgs.msg import Odometry 

import random
import math

# Converte um quaternion para ângulos de Euler (roll, pitch, yaw).
def euler_from_quaternion(quaternion):
    t0 = +2.0 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2])
    t1 = +1.0 - 2.0 * (quaternion[0]**2 + quaternion[1]**2)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0])
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
    t4 = +1.0 - 2.0 * (quaternion[1]**2 + quaternion[2]**2)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class Turtlebot_GoalManager(Node):

    def __init__(self):
        super().__init__('turtlebot_goal_manager')
        self.get_logger().info("Iniciado o Node do Gerenciador de Metas.")
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_variables(self):
        self.x = 0.0            # Posição atual do turtle
        self.y = 0.0            # Posição atual do turtle
        self.theta = 0.0        # Orientação atual do turtle

        self.x_goal = 0.0      # Posição objetivo
        self.y_goal = 0.0      # Posição objetivo

        self.distance_threshold = 0.01
        self.angle_threshold = 0.05

    def init_publisher(self):
        self.goal_publisher = self.create_publisher(Pose2D, '/goal', 10)

    def init_subscribers(self): # Inicializa os subscribers
        self.pose_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        # Extrai a posição do robô
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extrai a orientação do robô (em quaternion)
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Converte o quaternion para ângulos de Euler
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.theta = yaw
        dist_x = abs(self.x - self.x_goal)
        dist_y = abs(self.y - self.y_goal)

        # Verifica se o robô está parado (velocidade linear é zero).
        if dist_x < 0.01 and dist_y < 0.01:
            self.publish_new_goal()

    def publish_new_goal(self):
        goals = [
            Pose2D(x=-2.0, y=2.0), Pose2D(x=2.0, y=2.0), 
            Pose2D(x=-2.0, y=-2.0), Pose2D(x=2.0, y=-2.0)
        ]

        # Publica uma posição-objetivo aleatória
        goal_index = random.randint(0, len(goals) - 1)
        self.x_goal = goals[goal_index].x
        self.y_goal = goals[goal_index].y
        self.goal_publisher.publish(goals[goal_index])
        self.get_logger().info(f"Nova posição-objetivo publicada: {goals[goal_index]}")

def main(args=None):
    rclpy.init(args=args)
    turtlebot_goal_manager = Turtlebot_GoalManager()
    rclpy.spin(turtlebot_goal_manager)
    turtlebot_goal_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
