#!/bin/bash
# Сборка пакета участников и запуск решения.
#
#   GUI=true|false   — разрешены ли окна (rviz, cv2.imshow). На прогоне организаторов GUI=false.
#   Команда контейнера по умолчанию: ros2 launch solution start.launch.py
#   Для ручного режима в docker-compose раскомментируйте `command: bash`.
source /opt/ros/humble/setup.bash
cd /ros_ws

if [ -f src/solution/requirements.txt ]; then
    echo "[solution] pip install -r requirements.txt"
    pip3 install --no-cache-dir -q -r src/solution/requirements.txt
fi

echo "[solution] colcon build"
if ! colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    echo "[solution] ОШИБКА СБОРКИ — исправьте код и перезапустите: docker compose restart solution"
    exec sleep infinity
fi
source install/setup.bash

if [ "$1" = "ros2" ] && [ "$2" = "launch" ]; then
    exec "$@" gui:="${GUI:-true}"
fi
exec "$@"
