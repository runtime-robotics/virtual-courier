# Образ сцены: Webots R2025a + ROS 2 Humble (rclpy) + мир, робот и судья.
FROM cyberbotics/webots:R2025a-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# packages.ros.org иногда не резолвится на раннерах CI: повторяем update, пока индекс ROS не скачается.
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries \
 && apt-get update && apt-get install -y --no-install-recommends curl gnupg2 lsb-release ca-certificates procps \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list \
 && for i in 1 2 3 4 5; do apt-get update && apt-cache show ros-humble-ros-base >/dev/null 2>&1 && break; \
      echo "apt: индекс ROS не получен, попытка $i"; sleep 20; done \
 && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-rmw-fastrtps-cpp \
      python3-yaml \
 && rm -rf /var/lib/apt/lists/*

COPY scene /scene
COPY docker/scene/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /scene/generator/generate_world.py

WORKDIR /scene
ENTRYPOINT ["/entrypoint.sh"]
