# Design of a controller and monitoring system for a mobile robot based on ROS2

This project implements a differential drive mobile robot using ROS2 for
autonomous navigation and control.
My Project: https://youtu.be/Wr23Bm83ORE
## Features

- Differential drive robot
- SLAM mapping
- AMCL localization
- Navigation using Nav2
- Simulation in Gazebo
- Visualization in RViz

## System Architecture

ROS2 nodes:

- robot_state_publisher
- controller_node
- slam_toolbox
- nav2
- 
Extended Kalman Filter - EKF

## Hardware

- Jetson Orin NX
- STM32
- LiDAR
- IMU
- Encoder

## Simulation

Gazebo + RViz

## Run

```bash
colcon build
source install/setup.bash
ros2 launch mobile nav2.launch.py
