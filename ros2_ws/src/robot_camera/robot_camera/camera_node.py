import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

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

        # Настраиваем камеру
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if self.compressed:
            self.publisher = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        else:
            self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        self.timer = self.create_timer(1/fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
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
