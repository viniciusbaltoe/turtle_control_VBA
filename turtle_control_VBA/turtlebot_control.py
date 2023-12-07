import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose as TurtlePose

import math

def distancia_euclidiana(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calcular_angulo_entre_pontos(x1, y1, x2, y2):
    return math.atan2(y2-y1, x2-x1)

class TurtleController_VBA(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("Iniciado o Node de controle de Vinícius B. A.")
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


    def init_publisher(self):   # Inicializa os publishers
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def init_subscribers(self): # Inicializa os subscribers
        self.pose_subscriber = self.create_subscription(
            TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        self.goal_subscriber = self.create_subscription(
            Pose2D, '/goal', self.goal_callback, 10)

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.pub_callback()

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y  

    def pub_callback(self):
        twist_msg = Twist()

        if distancia_euclidiana(self.x, self.y, self.x_goal, self.y_goal) > self.distance_threshold:
            theta_goal = calcular_angulo_entre_pontos(self.x, self.y, self.x_goal, self.y_goal)
            if abs(self.theta - theta_goal) > self.angle_threshold:
                # Fase de rotação
                theta_error = theta_goal - self.theta

                # Ajuste do ângulo para o intervalo [-pi, pi]
                if theta_error > math.pi:
                    theta_error -= 2 * math.pi
                elif theta_error < -math.pi:
                    theta_error += 2 * math.pi

                angular_correction = theta_error
                linear_correction = 0.0
            else:
                # Fase de movimento linear
                angular_correction = 0.0
                x_error = distancia_euclidiana(self.x, self.y, self.x_goal, self.y_goal)
                linear_correction = x_error

            # Publica velocidade
            twist_msg.linear.x = linear_correction
            twist_msg.angular.z = angular_correction

        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        self.velocity_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController_VBA()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
