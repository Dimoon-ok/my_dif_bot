import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrackerNode(Node):
    def __init__(self):
        super().__init__('red_tracker_node')

        self.bridge=CvBridge()
        self.subscription=self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10
        )

        self.search_speed = 0.3

        self.bridge = CvBridge()

        self.publisher=self.create_publisher(Twist, 'cmd_vel',10)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.linear_speed = 0.1   # м/с
        self.angular_gain = 0.002 # коэффициент поворота

    def listener_callback(self,msg):
        frame=self.bridge.imgmsg_to_cv2(msg, 'brg8')
        hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1=cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2=cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask=mask1|mask2

        contours, _ =cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist=Twist()

        if contours:
            contour=max(contours, key=cv2.contourArea)
            x,y,w,h=cv2.BoundingRect(contour)
            cx=x+w//2
            cy=y+h//2

            height,width, _ =frame.shape
            error_x=cx-width//2

            twist.linear.x=self.linear_speed
            twist.angular.z=-error_x*self.angular_gain

            cv2.circle(frame, (cx,cy),5,(0,255,0),-1)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        else:
            twist.linear.x=self.linear_speed
            twist.angular.z=self.search_speed

        self.publisher.publish(twist)

def main():
    rclpy.init()
    node=TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
