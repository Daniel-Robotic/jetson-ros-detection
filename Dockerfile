# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM nvcr.io/nvidia/l4t-base:35.4.1

# Устанавливаем переменные окружения для CUDA
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# Устанавливаем рабочую директорию
WORKDIR /app/ws_ros2

# Устанавливаем необходимые зависимости
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-get update && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
        curl \
        gnupg2 \
        usbutils \
        python3-pip python3-dev \
        cuda \
        nvidia-tensorrt

# # Устанавливаем python3.9
# RUN apt-get update && \
#     add-apt-repository ppa:deadsnakes/ppa && \
#     apt-get remove -y python3.8 python3.8-* && \
#     apt-get update && \
#     apt-get install -y python3.9 python3.9-dev python3.9-venv python3-pip && \
#     apt-get clean

# Устанавливаем ROS 2 Foxy
RUN apt-get update && \
    curl -sL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \ 
        ros-foxy-desktop \
        python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*


# Установка python библиотек
RUN pip3 install torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 --index-url https://download.pytorch.org/whl/cu118 && \
    pip3 install numpy --upgrade && \
    pip3 install pyrealsense2 \
        colcon-common-extensions

COPY ./src /app/ws_ros2/

# Build project
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build"

CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]