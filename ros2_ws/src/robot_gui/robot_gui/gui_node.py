#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QDoubleSpinBox, QPushButton, QGroupBox)
from PyQt5.QtCore import QTimer
import pyqtgraph as pg


class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.pid_pub = self.create_publisher(Float32MultiArray, '/set_pid', 10)
        self.debug_sub = self.create_subscription(
            Float32MultiArray, '/robot_debug_info', self.debug_callback, 10)

        self.target_l = 0.0
        self.actual_l = 0.0
        self.target_r = 0.0
        self.actual_r = 0.0

    def send_pid(self, kp, ki, kd, kff):
        msg = Float32MultiArray()
        msg.data = [float(kp), float(ki), float(kd), float(kff)]
        self.pid_pub.publish(msg)
        self.get_logger().info(f"Published PID: {msg.data}")

    def debug_callback(self, msg):
        if len(msg.data) >= 4:
            self.target_l = msg.data[0]
            self.actual_l = msg.data[1]
            self.target_r = msg.data[2]
            self.actual_r = msg.data[3]


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Robot Control & PID Tuning")
        self.resize(900, 600)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # --- Группа настройки PID ---
        pid_group = QGroupBox("PID Coefficients")
        pid_layout = QHBoxLayout()

        # Значения по умолчанию из вашего скетча
        self.spin_kp = self.create_spinbox("Kp", 1.1)
        self.spin_ki = self.create_spinbox("Ki", 1.3)
        self.spin_kd = self.create_spinbox("Kd", 0.01)
        self.spin_kff = self.create_spinbox("Kff", 0.25)

        pid_layout.addWidget(QLabel("Kp:"))
        pid_layout.addWidget(self.spin_kp)
        pid_layout.addWidget(QLabel("Ki:"))
        pid_layout.addWidget(self.spin_ki)
        pid_layout.addWidget(QLabel("Kd:"))
        pid_layout.addWidget(self.spin_kd)
        pid_layout.addWidget(QLabel("Kff:"))
        pid_layout.addWidget(self.spin_kff)

        btn_send = QPushButton("Send PID")
        btn_send.clicked.connect(self.on_send_pid)
        pid_layout.addWidget(btn_send)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        # --- Графики ---
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle("Left Wheel Speed Response")
        self.plot_widget.setLabel('left', 'Speed', units='mm/s')
        self.plot_widget.setLabel('bottom', 'Time', units='ticks')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()

        self.curve_target = self.plot_widget.plot(pen='r', name='Target')
        self.curve_actual = self.plot_widget.plot(pen='g', name='Actual')

        layout.addWidget(self.plot_widget)

        # Буферы данных для графика
        self.data_limit = 300
        self.target_data = [0] * self.data_limit
        self.actual_data = [0] * self.data_limit

        # Таймер для обновления GUI и обработки событий ROS
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(50)  # 20 Hz

    def create_spinbox(self, name, val):
        sb = QDoubleSpinBox()
        sb.setRange(0.0, 100.0)
        sb.setSingleStep(0.01)
        sb.setDecimals(4)
        sb.setValue(val)
        return sb

    def on_send_pid(self):
        self.ros_node.send_pid(
            self.spin_kp.value(),
            self.spin_ki.value(),
            self.spin_kd.value(),
            self.spin_kff.value()
        )

    def update_loop(self):
        # Крутим ноду ROS, чтобы получить сообщения
        rclpy.spin_once(self.ros_node, timeout_sec=0)

        # Обновляем данные графиков
        self.target_data.append(self.ros_node.target_l)
        self.actual_data.append(self.ros_node.actual_l)

        if len(self.target_data) > self.data_limit:
            self.target_data.pop(0)
            self.actual_data.pop(0)

        self.curve_target.setData(self.target_data)
        self.curve_actual.setData(self.actual_data)


def main(args=None):
    rclpy.init(args=args)
    ros_node = RobotGuiNode()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
