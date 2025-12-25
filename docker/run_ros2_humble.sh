#!/bin/bash
xhost +local:docker

docker run -it \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/shm:/dev/shm \
  -v /dev:/dev \
  --device /dev/i2c-1 \
  -v ~/ros2_docker_data/ws:/root/ros2_ws \
  -v ~/adaptive_gps_slam_rover/ros2_ws/src:/root/ros2_ws/src \
  -v ~/ros2_docker_data/rviz:/root/rviz \
  -v ~/ros2_docker_data/bags:/root/bags \
  adaptive-rover

