import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/di/my_dif_bot/ros2_ws/install/my_dif_bot_description'
