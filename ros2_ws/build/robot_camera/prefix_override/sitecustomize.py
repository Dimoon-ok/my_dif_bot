import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/di/my_dif_bot/ros2_ws/install/robot_camera'
