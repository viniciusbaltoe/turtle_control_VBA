import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose as TurtlePose
from nav_msgs.msg import Odometry

import math

def distancia_euclidiana(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calcular_angulo_entre_pontos(x1, y1, x2, y2):
    return math.atan2(y2-y1, x2-x1)

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

class TurtlebotController_VBA(Node):

    def __init__(self):
        super().__init__('turtlebot_control')
        self.get_logger().info("Iniciado o Node de controle de Vinícius B. A.")
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


    def init_publisher(self):   # Inicializa os publishers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def init_subscribers(self): # Inicializa os subscribers
        self.pose_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(
            Pose2D, '/goal', self.goal_callback, 10)

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
    turtlebot_control = TurtlebotController_VBA()
    rclpy.spin(turtlebot_control)
    turtlebot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
