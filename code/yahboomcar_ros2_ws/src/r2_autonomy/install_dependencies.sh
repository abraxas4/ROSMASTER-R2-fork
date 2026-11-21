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
echo "OPTIONAL: frontier exploration (apt 에 ros-humble-explore-lite 없음):"
echo "  설치 절차는 r2_autonomy/INSTALL.md 참고"
echo "  (m-explore-ros2 main 클론 + Humble tf2_ros 패치 + explore_lite 만 빌드)"
echo ""
echo "====================================================================="
