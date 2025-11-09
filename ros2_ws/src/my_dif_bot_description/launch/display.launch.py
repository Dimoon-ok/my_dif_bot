from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Получение общего пути пакета learning_urdf.
    urdf_path = get_package_share_path('my_dif_bot_description')

    # Путь по умолчанию к URDF-модели и путь к конфигурационному файлу RViz.
    default_model_path = urdf_path / 'urdf/robot.urdf.xacro'
    default_rviz_config_path = urdf_path / 'rviz/urdf.rviz'
    
    # Объявление параметра запуска: включение joint_state_publisher_gui, по умолчанию false.
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    # Объявление параметра запуска: путь к модели, по умолчанию абсолютный путь к URDF-файлу.
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    # Объявление параметра запуска: путь к конфигурационному файлу RViz, по умолчанию абсолютный
    # путь к конфигурационному файлу RViz.
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # Определение параметра описания робота, использование xacro для преобразования URDF-файла
    # в параметр ROS.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Создание узла robot_state_publisher для публикации информации о состоянии робота.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # В зависимости от параметра gui запуск узла joint_state_publisher или узла
    # joint_state_publisher_gui.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')) # Если параметр gui равен false,
    # выполняется запуск.
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Если параметр gui равен true,
    # выполняется запуск.
    # Создание узла RViz с загрузкой указанного конфигурационного файла RViz.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    # Загрузка конфигурационного файла
    # с по­мощью параметра -rvizconfig.
    )
    
    # Возврат объекта LaunchDescription, содержащего все определенные действия и параметры запуска.
    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])