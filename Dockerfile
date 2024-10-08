# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM arm64v8/ros:foxy-ros-base-focal

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=foxy \
    BUILD_VERSION=0.16.1

RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        python3-dev \
        python3-colcon-common-extensions \
        build-essential \
        curl \
        git \
        wget \
        libopenblas-base \ 
        libopenmpi-dev \ 
        libomp-dev \
        libjpeg-dev \ 
        zlib1g-dev \ 
        libpython3-dev \ 
        libopenblas-dev \ 
        libavcodec-dev \ 
        libavformat-dev \ 
        libswscale-dev \
    && rm -rf /var/lib/apt/lists/*

COPY torch-v2.1.whl /tmp/

# Установка PyToroch с поддержкой GPU для Jetson
RUN git config --global http.postBuffer 104857600 && \
    git clone --branch v0.16.1 https://github.com/pytorch/vision /tmp/torchvision/ && \
    pip3 install --upgrade pip && \
    pip3 install pyrealsense2 && \
    pip3 install /tmp/torch-v2.1.whl && \
    python3 /tmp/torchvision/setup.py install --user



# Установка отдельных комронентов ROS
RUN apt-get update && \
    apt-get install -y \
        ros-foxy-rmw-cyclonedds-cpp \
        ros-foxy-desktop \
        ros-foxy-dev \
    && rm -rf /var/lib/apt/lists/*



# Сборка скопированного проекта
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build"

ENTRYPOINT ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]