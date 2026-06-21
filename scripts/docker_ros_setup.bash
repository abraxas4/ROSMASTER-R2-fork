#!/bin/bash
# Source ROS2 environments inside the Yahboom Docker container.
# Mounted at /root/docker_ros_setup.bash by run_docker.sh

source /opt/ros/foxy/setup.bash

if [[ -f /root/library_ws/install/setup.bash ]]; then
  source /root/library_ws/install/setup.bash
fi

if [[ -f /root/yahboomcar_ros2_ws/install/setup.bash ]]; then
  source /root/yahboomcar_ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
export ROBOT_TYPE="${ROBOT_TYPE:-r2}"
export RPLIDAR_TYPE="${RPLIDAR_TYPE:-4ROS}"
export CAMERA_TYPE="${CAMERA_TYPE:-astraplus}"