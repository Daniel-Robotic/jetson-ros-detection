# Команды перед исполнением Dockerfile
# git-lfs
# git lfs install
# git clone

# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM nvcr.io/nvidia/l4t-base:35.4.1

# Устанавливаем переменные окружения для CUDA
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=foxy \
    BUILD_VERSION=0.16.1 \
    PATH=/usr/local/cuda/bin:${PATH}
    
WORKDIR /app/ros2_ws

# Устанавливаем необходимые зависимости
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-get update && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
        curl \
        git \
        gnupg2 \
        usbutils \
        python3-pip python3-dev \
        cuda \
        nvidia-tensorrt \
        libopenblas-dev

# Устанавливаем ROS 2 Foxy
RUN apt-get update && \
    curl -sL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \ 
        ros-foxy-desktop \
        python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# Копирования
COPY torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl /tmp/
COPY ./vision/* /tmp/vision/

# Установка PyToroch с поддержкой GPU для Jetson
RUN pip3 install --upgrade pip && \
    pip3 install --upgrade numpy && \
    pip3 install pyrealsense2 colcon-common-extensions && \
    pip3 install /tmp/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    # pip3 install git+https://github.com/pytorch/vision.git@release/0.16
    pip3 install /tmp/vision/

COPY ./src /app/ros2_ws/src/

# Сборка скопированного проекта
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build" && \
    rm -r build log src /tmp/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

# ENTRYPOINT ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]
ENTRYPOINT [ "bash", "-c", "source /app/ros2_ws/install/setup.bash && ros2 run test_packages talker" ]