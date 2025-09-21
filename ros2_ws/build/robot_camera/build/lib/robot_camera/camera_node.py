import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('compressed', False)

        # Читаем значения из параметров
        cam_index = self.get_parameter('camera_index').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.compressed = self.get_parameter('compressed').value

        self.get_logger().info(f"Camera index: {cam_index}")
        self.get_logger().info(f"Resolution: {width}x{height}")
        self.get_logger().info(f"FPS: {fps}")
        self.get_logger().info(f"Compressed: {self.compressed}")

        # Настраиваем камеру
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.prev_time = time.time()
        self.fps = 0
        self.frame_count = 0
        self.fps_update_interval = 1
        self.last_fps_update = time.time()

        if self.compressed:
            self.publisher = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
            self.get_logger().info("Publishing compressed images")
        else:
            self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
            self.get_logger().info("Publishing raw images")
        self.bridge = CvBridge()

        self.timer = self.create_timer(1/fps, self.timer_callback)

    def calculate_fps(self):
        """Рассчитывает FPS"""
        current_time = time.time()
        self.frame_count += 1

        # Обновляем FPS каждые fps_update_interval секунд
        if current_time - self.last_fps_update >= self.fps_update_interval:
            self.fps = self.frame_count / (current_time - self.last_fps_update)
            self.frame_count = 0
            self.last_fps_update = current_time

        return self.fps

    def draw_fps_on_frame(self, frame, fps):
        """Рисует FPS на кадре"""
        fps_text = f"FPS: {fps:.1f}"

        # Настройки текста
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_color = (0, 255, 0)  # зеленый цвет
        font_thickness = 2
        position = (10, 30)  # левый верхний угол

        # Добавляем текст на изображение
        cv2.putText(frame, fps_text, position, font, font_scale,
                    font_color, font_thickness, cv2.LINE_AA)

        # Добавляем полупрозрачный фон для лучшей читаемости
        text_size = cv2.getTextSize(fps_text, font, font_scale, font_thickness)[0]
        bg_position = (position[0] - 5, position[1] - text_size[1] - 5)
        bg_size = (text_size[0] + 10, text_size[1] + 10)

        # Создаем полупрозрачный прямоугольник
        overlay = frame.copy()
        cv2.rectangle(overlay,
                        (bg_position[0], bg_position[1]),
                        (bg_position[0] + bg_size[0], bg_position[1] + bg_size[1]),
                        (0, 0, 0), -1)

        # Накладываем полупрозрачный фон
        alpha = 0.6  # прозрачность
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            current_fps = self.calculate_fps()
            self.draw_fps_on_frame(frame, current_fps)
            #msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            if self.compressed:
                msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            else:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("Frame not captured")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
