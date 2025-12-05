#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray

import tf2_ros
import serial
import re
from transforms3d.euler import euler2quat
from geometry_msgs.msg import Pose2D
import glob
import time


class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')

        # подписка на cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # публикация в odom
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )

        self.set_pose_sub = self.create_subscription(
            Pose2D,
            '/set_pose',
            self.set_pose_callback,
            10
        )

        # Подписка на изменение PID коэффициентов из GUI
        self.pid_sub = self.create_subscription(
            Float32MultiArray,
            '/set_pid',
            self.pid_callback,
            10
        )

        # Публикация отладочных данных (скорости) для графиков
        self.debug_pub = self.create_publisher(
            Float32MultiArray,
            '/robot_debug_info',
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # подключение к serial порту
        self.ser = self.find_robot_serial()
        if not self.ser:
            raise RuntimeError("Не удалось найти контроллер по Serial")
        self.get_logger().info(f"Opened serial port: {self.ser.port}")

        # Обновленная регулярка для парсинга всей строки телеметрии
        # Формат ESP32: POS X=... Y=... Th=... ENC ... SPD L=... R=... Target L=... R=...
        self.telemetry_pattern = re.compile(
            r'POS X=([\-\d\.]+) Y=([\-\d\.]+) Th=([\-\d\.]+).*?'
            r'SPD L=([\-\d\.]+) mm/s R=([\-\d\.]+) mm/s.*?'
            r'Target L=([\-\d\.]+) mm/s R=([\-\d\.]+) mm/s'
        )

        self.read_serial_timer = self.create_timer(
            0.05, self.read_serial_timer_callback)

    def find_robot_serial(self):
        ports = glob.glob("/dev/serial/by-path/*")
        for port in ports:
            try:
                ser = serial.Serial(port, 115200, timeout=0.1)
                ser.write(b"PING\n")
                time.sleep(0.05)
                if b"ROBOT_001" in ser.readline():
                    return ser
                ser.close()
            except:
                pass
        return None

    def pid_callback(self, msg: Float32MultiArray):
        # Ожидаем массив [Kp, Ki, Kd, Kff]
        if len(msg.data) >= 4:
            kp, ki, kd, kff = msg.data[:4]
            command = f"SET_COEFF {kp:.4f} {ki:.4f} {kd:.4f} {kff:.4f}\n"
            try:
                self.ser.write(command.encode())
                self.get_logger().info(f"Sent PID update: {command.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to write PID to serial: {e}")

    def cmd_vel_callback(self, msg: Twist):
        linear_velocity = msg.linear.x * 1000.0  # м/с -> мм/с
        angular_velocity = msg.angular.z         # рад/с
        command = f"SET_ROBOT_VELOCITY {linear_velocity:.2f} {angular_velocity:.2f}\n"
        try:
            self.ser.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial: {e}")

    def set_pose_callback(self, msg: Pose2D):
        x = msg.x * 1000
        y = msg.y * 1000
        command = f"SET_POSE {x:.2f} {y:.2f} {msg.theta:.2f}\n"
        try:
            self.ser.write(command.encode())
            self.get_logger().info(f"Sent command: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial: {e}")

    def read_serial_timer_callback(self):
        # Считываем всё, что накопилось в буфере (т.к. timeout=0.1)
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()

            if not line:
                return

            match = self.telemetry_pattern.search(line)
            if match:
                x_mm = float(match.group(1))  # мм
                y_mm = float(match.group(2))  # мм
                th = float(match.group(3))  # рад

                spd_l = float(match.group(4))
                spd_r = float(match.group(5))
                tgt_l = float(match.group(6))
                tgt_r = float(match.group(7))

                # Публикуем отладочные данные: [TargetL, ActualL, TargetR, ActualR]
                debug_msg = Float32MultiArray()
                debug_msg.data = [tgt_l, spd_l, tgt_r, spd_r]
                self.debug_pub.publish(debug_msg)

                # Переводим мм -> м
                x = x_mm / 1000.0
                y = y_mm / 1000.0

                # Формируем Odometry
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'

                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = 0.0

                # Преобразуем угол в кватернион
                # returns (w, x, y, z) - такая у transforms3d специфика
                q = euler2quat(0.0, 0.0, th)
                odom_msg.pose.pose.orientation.w = q[0]
                odom_msg.pose.pose.orientation.x = q[1]
                odom_msg.pose.pose.orientation.y = q[2]
                odom_msg.pose.pose.orientation.z = q[3]

                # Публикуем в топик /odom
                self.odom_pub.publish(odom_msg)

                # Одновременно шлём TF odom -> base_link
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation.w = q[0]
                t.transform.rotation.x = q[1]
                t.transform.rotation.y = q[2]
                t.transform.rotation.z = q[3]

                self.tf_broadcaster.sendTransform(t)

                self.get_logger().info(
                    f"ODOM: X={x:.3f} Y={y:.3f} Th={th:.3f}")
            else:
                self.get_logger().debug(f"Unknown line: {line}")

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to read from serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
