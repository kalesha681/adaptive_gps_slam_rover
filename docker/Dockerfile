FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-serial-driver \
    ros-humble-rviz2 \
    usbutils \
    minicom \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws
RUN mkdir src

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
