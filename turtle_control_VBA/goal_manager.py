import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose as TurtlePose

import random

class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')
        self.get_logger().info("Iniciado o Node do Gerenciador de Metas.")
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_variables(self):
        self.x = 0.0            # Posição atual do turtle
        self.y = 0.0            # Posição atual do turtle
        self.theta = 0.0        # Orientação atual do turtle

        self.x_goal = 5.54      # Posição objetivo
        self.y_goal = 5.54      # Posição objetivo

        self.distance_threshold = 0.01
        self.angle_threshold = 0.05

    def init_publisher(self):
        self.goal_publisher = self.create_publisher(Pose2D, '/goal', 10)

    def init_subscribers(self): # Inicializa os subscribers
        self.pose_subscriber = self.create_subscription(
            TurtlePose, '/turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        # Verifica se a tartaruga está parada para enviar outro destino (velocidade linear e angular são zero).
        if msg.linear_velocity == 0.0 and msg.angular_velocity == 0.0:
            self.publish_new_goal()

    def publish_new_goal(self):
        goals = [
            Pose2D(x=1.0, y=10.0), Pose2D(x=1.0, y=1.0), 
            Pose2D(x=10.0, y=1.0), Pose2D(x=10.0, y=10.0)
        ]

        # Publica uma posição-objetivo aleatória
        goal_index = random.randint(0, len(goals) - 1)
        self.x_goal = goals[goal_index].x
        self.y_goal = goals[goal_index].y
        self.goal_publisher.publish(goals[goal_index])
        self.get_logger().info(f"Nova posição-objetivo publicada: {goals[goal_index]}")

def main(args=None):
    rclpy.init(args=args)
    goal_manager = GoalManager()
    rclpy.spin(goal_manager)
    goal_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
