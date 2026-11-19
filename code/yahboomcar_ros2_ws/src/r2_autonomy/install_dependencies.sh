#!/bin/bash
# Install dependencies for r2_autonomy package on Jetson Orin Nano

set -e

echo "Installing r2_autonomy dependencies for ROS 2 Humble..."

# Update package lists
sudo apt update

# Core Nav2 packages
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-behaviors \
    ros-humble-navigation2

# SLAM packages
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros

# Localization
sudo apt install -y \
    ros-humble-robot-localization

# Additional utilities
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-xacro

echo ""
echo "====================================================================="
echo "Core dependencies installed successfully!"
echo ""
echo "OPTIONAL: To install frontier exploration (m-explore-ros2):"
echo "  cd ~/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws/src"
echo "  git clone -b humble https://github.com/robo-friends/m-explore-ros2.git"
echo "  cd .."
echo "  colcon build --packages-select explore_lite --symlink-install"
echo ""
echo "====================================================================="
