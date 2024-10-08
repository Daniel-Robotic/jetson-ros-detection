# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM arm64v8/ros:foxy-ros-base-focal

RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        python3-dev \
        python3-colcon-common-extensions \
        build-eesential \
        curl \
        git \
    && rm -rf /var/lib/apt/lists/*

# Установка PyToroch с поддержкой GPU для Jetson
RUN pip3 install --upgrade pip && \
    pip3 install pyrealsense2 && \
    pip3 install torch==1.14.0a0+nv23.04 \
                 torchvision=0.15.0a0+nv23.04 \
                --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v502


# Установка отдельных комронентов ROS
RUN apt-get update && \
    apt-get install -y \
        ros-foxy=rmw-cyclonedds-cpp \
        ros-foxy-desktop \
        ros-foxy-dev \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=foxy

# Сборка скопированного проекта
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build"

ENTRYPOINT ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]