# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM nvcr.io/nvidia/l4t-base:35.3.1

# Настройка переменных окружения
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=foxy

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-foxy-desktop \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyrealsense2 \
                 torch torchvision torchaudio \
                 colcon-common-extensions

# Установка дополнительных пакетов ROS Foxy
RUN source /opt/ros/foxy/setup.bash && \
    apt-get update && apt-get install -y \
    ros-foxy-rmw-cyclonedds-cpp

# Копирования
COPY ./src /app/ws_ros2/

# Сборка скопированного проекта
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build"

ENTRYPOINT ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]