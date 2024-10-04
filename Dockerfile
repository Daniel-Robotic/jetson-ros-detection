FROM osrf/ros:foxy-desktop-focal
# FROM arm64v8/foxy-ros-base-focal

WORKDIR /apps/ros2_ws/
# COPY ./setup.bash /apps/ros2_ws/setup.bash
# RUN chmod +x /apps/ros2_ws/setup.bash \
    # && /apps/ros2_ws/setup.bash
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-foxy-rmw-cyclonedds-cpp \
    # && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    # && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    # && sudo apt-get update -y \
    # && sudo apt-get install ros-foxy-librealsense2* -y \
    # && sudo apt-get install ros-foxy-realsense2-* -y \
    && pip3 install pyrealsense2 \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c "source /opt/ros/foxy/setup.bash"

COPY ./ /apps/ros2_ws/

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select calibration_interfaces calibration" \
    && sudo rm -r build log src 
    # && /bin/bash -c "source /apps/ros2_ws/install/setup.sh"

# CMD ["bash", "-c", "source /apps/ros2_ws/install/setup.bash && ros2 run test_packages talker"]
CMD [ "bash", "-c",  "source /apps/ros2_ws/install/setup.bash && ros2 run calibration calibration --ros-args -p image_folder:='/apps/ros2_ws/calibration_images'"]
# ENTRYPOINT [ "bash", "-c", "source /apps/ros2_ws/install/setup.bash && exec ros2 run test_packages talker" ]