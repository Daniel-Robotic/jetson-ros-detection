# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM nvcr.io/nvidia/l4t-base:r36.2.0

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

# Устанавливаем ROS 2 Foxy
RUN apt-get update && \
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y \ 
        ros-humble-desktop \
        python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*


# Установка python библиотек
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 && \
    pip3 install numpy --upgrade && \
    pip3 install pyrealsense2 \
                 colcon-common-extensions

# Копирования файлов
COPY ./src /app/ws_ros2/

# Сборка скопированного проекта
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Заупск проекта
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"]