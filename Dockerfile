# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM nvcr.io/nvidia/l4t-base:35.4.1

# Устанавливаем переменные окружения для CUDA
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# Устанавливаем рабочую директорию
WORKDIR /app/ws_ros2

# Устанавливаем необходимые зависимости
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    usbutils \
    && rm -rf /var/lib/apt/lists/*


# Устанавливаем ROS 2 Foxy
RUN apt-get update && \
    apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-foxy-desktop python3-argcomplete ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# Скачиваем и устанавливаем CUDA
RUN apt-get update && apt-get install -y \
    cuda-toolkit-11.4 \
    cuda \
    libcudnn8 \
    libcudnn8-dev \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем необходимые зависимости для ROS 2
RUN apt-get update && apt-get install -y \
    ros-foxy-librealsense2* \
    ros-foxy-realsense2-* \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем pyrealsense2 через apt
RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/librealsense

# Установка python библиотек
RUN pip3 install --upgrade pip && \
    pip3 install \ 
        colcon-common-extensions \
        torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 && \
    pip3 install numpy --upgrade

COPY ./src /app/ws_ros2/

# Build project
RUN bash -c "source /opt/ros/foxy/setup.bash && colcon build"

CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]