from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_camera')

    camera_params_file = os.path.join(pkg_path, 'config', 'camera_params.yaml')
    cv_params_file = os.path.join(pkg_path, 'config', 'cv_params.yaml')

    camera_node = Node(
        package='robot_camera',
        executable='camera_node',
        name='camera_node',
        parameters=[camera_params_file],
        output='screen'
    )

    cv_node = Node(
        package='robot_camera',
        executable='object_follower_node',
        name='object_follower_node',
        parameters=[cv_params_file],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        cv_node,
    ])
