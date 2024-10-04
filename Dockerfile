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

# Добавляем репозиторий ROS 2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Устанавливаем ROS 2 Foxy
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем необходимые Python-пакеты
RUN pip3 install --upgrade pip

# Устанавливаем colcon для сборки ROS 2 пакетов
RUN pip3 install colcon-common-extensions

# Скачиваем и устанавливаем CUDA Toolkit
RUN apt-get update && apt-get install -y \
    cuda-toolkit-10-2 \
    && rm -rf /var/lib/apt/lists/*

    # Устанавливаем необходимые зависимости для ROS 2 и CUDA
RUN apt-get update && apt-get install -y \
    libcudnn8 \
    libcudnn8-dev \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]