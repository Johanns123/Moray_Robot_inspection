import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')


        self.pos = None

        # Subscrição ao tópico da câmera
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            10
        )

        # Subscrição ao tópico de odometria para pegar velocidade
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

         # Publisher para mover o robô
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para enviar comandos de movimento (a cada 0.5s)
        self.timer = self.create_timer(0.5, self.move_robot)

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Converte imagem ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_resized = cv2.resize(cv_image, (800, 600))  # Largura, Altura
            cv2.imshow("Camera View", cv_resized)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")

    def odom_callback(self, msg):
        self.pos = msg.pose.pose.position
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        # Imprime velocidade linear no terminal
        self.get_logger().info(f"Velocidade linear: x={linear.x:.3f}, w={angular.z:.3f}")
        self.get_logger().info(
            f"[move_robot] Posição atual: x={self.pos.x:.2f}, y={self.pos.y:.2f}"
        )

    def move_robot(self):
        

        # Você pode usar self.pos aqui também
        if self.pos.x > 2:
            twist = Twist()
            twist.linear.x = -2.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
        
        elif self.pos.x < -2:
            twist = Twist()
            twist.linear.x = 2.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    viewer = CameraViewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
