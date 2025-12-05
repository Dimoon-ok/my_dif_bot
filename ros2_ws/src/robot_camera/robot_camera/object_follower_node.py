import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectFollowerNode(Node):
    def __init__(self):
        super().__init__('object_follower_node')

        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('stop_area', 25000.0)

        # Значения задаются как списки [H, S, V]
        self.declare_parameter('hsv_lower1', [0, 120, 70])
        self.declare_parameter('hsv_upper1', [10, 255, 255])
        self.declare_parameter('hsv_lower2', [170, 120, 70])
        self.declare_parameter('hsv_upper2', [180, 255, 255])

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stop_area = self.get_parameter('stop_area').value

        # Читаем пороги из параметров
        self.hsv_lower1 = np.array(self.get_parameter(
            'hsv_lower1').value, dtype=np.uint8)
        self.hsv_upper1 = np.array(self.get_parameter(
            'hsv_upper1').value, dtype=np.uint8)
        self.hsv_lower2 = np.array(self.get_parameter(
            'hsv_lower2').value, dtype=np.uint8)
        self.hsv_upper2 = np.array(self.get_parameter(
            'hsv_upper2').value, dtype=np.uint8)

        self.get_logger().info(f"Linear_speed: {self.linear_speed}")
        self.get_logger().info(f"Angular_speed: {self.angular_speed}")
        self.get_logger().info(f"Stop_area: {self.stop_area}")
        # self.get_logger().info(
        #     f"HSV1: {self.hsv_lower1.tolist()} - {self.hsv_upper1.tolist()}")
        # self.get_logger().info(
        #     f"HSV2: {self.hsv_lower2.tolist()} - {self.hsv_upper2.tolist()}")

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        self.get_logger().info("Object Follower Node Started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        h, w, _ = cv_image.shape

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Используем пороги из параметров (два диапазона, т.к. красный вокруг 0/180 по H)
        mask1 = cv2.inRange(hsv, self.hsv_lower1, self.hsv_upper1)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.add(mask1, mask2)

        # Убираем шум
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Поиск контуров
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            # Берем самый большой контур
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:  # Фильтр совсем мелких бликов
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])

                    # Пропорциональное управление поворотом
                    error_x = cx - w / 2
                    twist.angular.z = - \
                        float(error_x) / (w / 2) * self.angular_speed * 2.0

                    # Управление движением вперед
                    if area < self.stop_area:
                        twist.linear.x = self.linear_speed
                    else:
                        twist.linear.x = 0.0  # Стоп, мы близко
            else:
                # Объект слишком маленький или это шум -> ищем
                twist.angular.z = self.angular_speed
        else:
            # Объект не найден -> вращаемся на месте
            twist.angular.z = self.angular_speed

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
