import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import heapq
import math
from scipy.spatial.transform import Rotation as R

# Constants
LIMIT_ANGULAR_SPEED = 1.0
LIMIT_LINEAR_SPEED = 0.5


def euler_from_quaternion(quat):
    r = R.from_quat([quat[0], quat[1], quat[2], quat[3]])
    return r.as_euler('xyz', degrees=False)


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)

        self.initial_pose_offset = (-10.0, 0.0)  # ajuste conforme posição inicial do robô
        self.initial_orientation_offset = 0.0

        self.bridge = CvBridge()
        self.current_pose = None
        self.destinations = [(0.0, 0.0), (-1.0, 0.0)]
        self.destination_index = 0
        self.desired_phi = 0.0
        self.previous_x = 0
        self.previous_y = 0
        self.angular_error_k1 = 0
        self.distance_error_k1 = 0
        self.uk_ang_k1 = 0
        self.uk_disp_k1 = 0
        self.counter = 0

        self.timer = self.create_timer(0.1, self.move_robot)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            resized_image = cv2.resize(cv_image, (800, 600))
            cv2.imshow("Camera View", resized_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        # Transformação para coordenadas globais considerando offset e orientação
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        global_x = self.initial_pose_offset[0] + (odom_x * cos_yaw - odom_y * sin_yaw)
        global_y = self.initial_pose_offset[1] + (odom_x * sin_yaw + odom_y * cos_yaw)

        self.current_pose = (global_x, global_y, yaw)

    def move_robot(self):
        if self.current_pose is not None and self.destination_index < len(self.destinations):
            curr_x, curr_y, phi = self.current_pose
            goal_x, goal_y = self.destinations[self.destination_index]

            u_x = goal_x - curr_x
            u_y = goal_y - curr_y

            self.desired_phi = math.atan2(u_y, u_x)
            angular_error = math.atan2(math.sin(self.desired_phi - phi), math.cos(self.desired_phi - phi))
            distance_error = math.hypot(u_x, u_y)

            uk_ang = 1.4939 * angular_error - 1.442808 * self.angular_error_k1 + self.uk_ang_k1
            uk_disp = 2.8154 * distance_error - 2.719113 * self.distance_error_k1 + self.uk_disp_k1

            uk_ang = max(min(uk_ang, LIMIT_ANGULAR_SPEED), -LIMIT_ANGULAR_SPEED)
            uk_disp = max(min(uk_disp, 0.5 if abs(angular_error) > 0.03 else 1.8), -0.5)

            self.angular_error_k1 = angular_error
            self.distance_error_k1 = distance_error
            self.uk_ang_k1 = uk_ang
            self.uk_disp_k1 = uk_disp

            twist = Twist()

            if abs(angular_error) > math.radians(2):
                twist.linear.x = 0.0
                twist.angular.z = uk_ang
            else:
                twist.linear.x = uk_disp
                twist.angular.z = uk_ang

            if distance_error <= 0.1:
                self.velocity_pub.publish(Twist())
                self.get_logger().info('Reached target position.')
                self.get_logger().info(f"Final position: x={curr_x:.2f}, y={curr_y:.2f}, phi={phi:.2f}")
                self.destination_index += 1
                return

            #self.velocity_pub.publish(twist)

            if self.counter < 10:
                self.counter += 1
            else:
                self.get_logger().info(f"Current pose: x={curr_x:.2f}, y={curr_y:.2f}, phi={phi:.2f}")
                self.counter = 0


def read_map(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        return [list(map(float, line.strip().split())) for line in lines]


def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def a_star(map_data, start, goal):
    rows, cols = len(map_data), len(map_data[0])
    moves = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        for dx, dy in moves:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                new_cost = cost_so_far[current] + map_data[int(neighbor[0])][int(neighbor[1])]

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

    if goal not in came_from:
        return None, float('inf')

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path, cost_so_far[goal]


def simplify_path(path):
    if not path:
        return []

    simplified = [path[0]]
    current_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])

    for i in range(1, len(path) - 1):
        next_dir = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if next_dir != current_dir:
            simplified.append(path[i])
            current_dir = next_dir

    simplified.append(path[-1])
    return simplified


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    controller.destinations = [(4, 0), (4, -4), (-3, -4), (-3, -7), (6, -7), (-10, 0)]

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


##coordinates
'''
[-1,0]
[7,0]
[6,-4]
[-3,-4]
[-3,-7]
[6,-7]
[-10, 0]
'''
