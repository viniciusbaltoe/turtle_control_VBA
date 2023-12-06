import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Pose2D, Twist
from turtlesim.msg import Pose as TurtlePose

class TurtleController_VBA(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("Iniciado o Node de controle do Vinícius B. A.")
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_variables(self):
        self.x = 0.0          # Posição atual do turtle
        self.x_goal = None    # Posição objetivo
        self.k_omega = 1.0    # Ganho do controlador angular

    def init_publisher(self):   # Inicializa os publishers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)   # cmd_vel
        self.timer = self.create_timer(0.1, self.pub_callback)

    def init_subscribers(self): # Inicializa os subscribers
        self.pose_subscriber = self.create_subscription(
            TurtlePose, '/turtle1/pose', self.pose_callback, 10)                         # pose
        self.goal_subscriber = self.create_subscription(
            Pose2D, '/goal', self.goal_callback, 10)                                    # goal

    def pose_callback(self, msg):
        self.x = msg.x

    def goal_callback(self, msg):
        #print(msg.x)
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta

    def pub_callback(self):
        if self.x_goal is not None:
            # Calcula o erro
            x_error = self.x_goal - self.x

            # Calcula a correção
            omega = self.k_omega * x_error

            # Publica velocidade
            twist_msg = Twist()
            twist_msg.angular.z = omega
            self.velocity_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController_VBA()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()