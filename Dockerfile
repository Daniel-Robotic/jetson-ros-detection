FROM amd64/ubuntu:20.04
# FROM arm64v8/ubuntu:20.04

WORKDIR /app/ros2_ws

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install Base packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    git \
    curl \
    software-properties-common

# Install ROS
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt upgrade \
    && apt install -y ros-foxy-desktop \ 
        python3-argcomplete \
        ros-foxy-ros-base \
        python3-argcomplete \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c "source /opt/ros/foxy/setup.bash"

CMD ["bash", "-c", "source /apps/ros2_ws/install/setup.bash && ros2 run demo_nodes_cpp talker"]