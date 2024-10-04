FROM nvidia/cuda-arm64:11.4.0-runtime-ubuntu20.04

ENV ROS_DISTRO=foxy
ENV TZ=Asia/Vladivostok
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

WORKDIR /app/ros2_ws

# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone


# Install Base packages
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/7fa2af80.pub \ 
    && apt-get update \ 
    && apt-get install -y --no-install-recommends \
    tzdata \
    build-essential \
    cmake \
	git \
    libbullet-dev \
    libpython3-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-rosinstall-generator \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    python3-pip \
    python3-dev \
    git \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install ROS
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt upgrade \
    && apt install -y ros-foxy-desktop \ 
        python3-argcomplete \
        ros-foxy-ros-base \
        python3-argcomplete \
        ros-dev-tools \
        ros-foxy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c "source /opt/ros/foxy/setup.bash"

CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"]