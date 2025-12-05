Robot GUI — запуск GUI на компьютере через rosbridge (Raspberry Pi — ROS2, ПК — Windows/Ubuntu)

Коротко:
- На Raspberry Pi остаётся ROS2 и все ноды робота.
- rosbridge_server на Pi предоставляет WebSocket/TCP (по умолчанию порт 9090).
- GUI запускается на ПК и подключается к Pi через roslibpy (WebSocket).

1) На Raspberry Pi (Jazzy)
- Убедитесь, что rosbridge_server установлен:
  - sudo apt update
  - sudo apt install ros-jazzy-rosbridge-server
- Запустите rosbridge (пример WebSocket):
  - ros2 run rosbridge_server rosbridge_websocket --port 9090
  - или TCP: ros2 run rosbridge_server rosbridge_tcp --port 9090
- Убедитесь, что ваши ROS-ноды робота запущены и публикуют/подписываются на нужные темы (/set_pid, /robot_debug_info).

2) На ПК (Windows или Ubuntu) — зависимости
- Установите Python 3.8+.
- Рекомендуется виртуальное окружение (venv).
- Установите зависимости:
  - pip install roslibpy PyQt5 pyqtgraph

3) Запуск GUI на ПК (подключение к Pi)
- Клонируйте/скопируйте ваш workspace на ПК или просто скопируйте папку robot_gui/robot_gui/gui_node.py.
- Запустите:
  - python gui_node.py --bridge <PI_IP>:9090
  - пример: python gui_node.py --bridge 192.168.1.10:9090
- GUI подключится к rosbridge и будет публиковать /set_pid и подписываться на /robot_debug_info.

4) Примечания и отладка
- Проверка rosbridge: с Pi можно сделать curl/ws check; из ПК подключитесь с roslibpy клиентом.
- Фаервол: откройте порт 9090 на Pi, если требуется.
- Безопасность: rosbridge по умолчанию не шифрует трафик — используйте внутри безопасной локальной сети.
- Задержки: возможны небольшие задержки через rosbridge, но обычно достаточно для GUI/тюнинга PID.
- На Ubuntu установка PyQt5/pyqtgraph чаще проще; на Windows убедитесь, что PyQt5 устанавливается корректно (pip wheel).

5) Альтернативы
- Запуск GUI непосредственно на Pi (ros2 run robot_gui gui_node) — если хотите полностью локально.
- Установка полного ROS2 на ПК (Humble/Jazzy и совпадение дистрибутива) — тогда GUI можно запускать как rclpy-ноду без rosbridge.

Если нужно, могу добавить короткий скрипт запуска или пример systemd unit для rosbridge на Raspberry Pi.
