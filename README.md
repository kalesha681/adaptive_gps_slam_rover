# Adaptive GPS + SLAM Rover

A dual-mode autonomous mobile robot that navigates:
- Outdoors using GPS + EKF
- Indoors using LiDAR SLAM (ROS2 Nav2)

Hardware:
- Raspberry Pi 5 (ROS2 Humble)
- Arduino Mega (real-time motor + sensor controller)
- YDLidar (2D LiDAR)
- u-blox M8N GPS
- BNO055 IMU
- BTS7960 motor driver

This repository contains:
- ROS2 workspace (ros2_ws/)
- Arduino firmware (firmware/)
- System documentation (docs/)
