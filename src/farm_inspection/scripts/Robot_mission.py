import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        # Subscrição ao tópico da câmera
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            10
        )
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
