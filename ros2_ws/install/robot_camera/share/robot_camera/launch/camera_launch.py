from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Получаем путь к папке пакета
    pkg_path = get_package_share_directory('robot_camera')

    # Формируем полный путь к YAML-файлу
    params_file = os.path.join(pkg_path, 'config', 'camera_params.yaml')

    return LaunchDescription([
        Node(
            package='robot_camera',
            executable='camera_node',
            name='camera_node',
            parameters=[params_file],
            output='screen'
        ),
    ])