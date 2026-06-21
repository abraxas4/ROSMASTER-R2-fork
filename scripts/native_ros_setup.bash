#!/bin/bash
# Native ROS2 Humble environment for sensor checks and mapping.
# library_ws binaries are built for the Jetson host (GLIBC 2.35), not Foxy Docker.

set +u
source /opt/ros/humble/setup.bash

if [[ -f "${LIBRARY_WS_HOST}/install/setup.bash" ]]; then
  source "${LIBRARY_WS_HOST}/install/setup.bash"
fi

if [[ -f "${NATIVE_WS_HOST}/install/setup.bash" ]]; then
  source "${NATIVE_WS_HOST}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
export ROBOT_TYPE="${ROBOT_TYPE:-r2}"
export RPLIDAR_TYPE="${RPLIDAR_TYPE:-4ROS}"
export CAMERA_TYPE="${CAMERA_TYPE:-astraplus}"