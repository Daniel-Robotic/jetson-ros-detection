# Используем базовый образ NVIDIA L4T с поддержкой CUDA
FROM nvcr.io/nvidia/l4t-base:r32.6.1

# Устанавливаем переменные окружения для ROS 2
ENV ROS_DISTRO=foxy
ENV ROS_PYTHON_VERSION=3

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
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*


# Устанавливаем ROS 2 Foxy
RUN apt-get update \
    apt install -y software-properties-common \
    apt-get install -y curl \
    add-apt-repository universe \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get install -y \
    ros-foxy-desktop python3-argcomplete ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Скачиваем и устанавливаем CUDA Toolkit
RUN apt-get update && apt-get install -y \
    cuda-toolkit-10-2 \
    && rm -rf /var/lib/apt/lists/*

    # Устанавливаем необходимые зависимости для ROS 2 и CUDA
RUN apt-get update && apt-get install -y \
    libcudnn8 \
    libcudnn8-dev \
    ros-foxy-librealsense2* \
    ros-foxy-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

# Установка python библио
RUN pip3 install --upgrade pip && \
    pip3 install \ 
        colcon-common-extensions \
        pyrealsense2 \
        numpy

COPY ./src /app/ws_ros2/

RUN source /opt/ros/foxy/setup.bash && \
    colcon build

CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]