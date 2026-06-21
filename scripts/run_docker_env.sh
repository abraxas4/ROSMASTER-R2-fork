#!/bin/bash
# Shared Docker environment for ROSMASTER R2 scripts

USER_HOME="$(eval echo ~$USER)"
GIT_REPO_PATH="${GIT_REPO_PATH:-${USER_HOME}/ROSMASTER-R2-fork}"
WORKSPACE_HOST="${GIT_REPO_PATH}/code/yahboomcar_ros2_ws"
# Yahboom prebuilt SLAM/lidar packages (slam_gmapping, sllidar, etc.)
LIBRARY_WS_HOST="${LIBRARY_WS_HOST:-${USER_HOME}/yahboomcar_ros2_ws/software/library_ws}"

resolve_docker_image() {
    local preferred=(
        "yahboomtechnology/ros-foxy:4.0.7R2"
        "yahboomtechnology/ros-foxy:4.0.7"
        "yahboomtechnology/ros-foxy:3.9.1"
        "yahboomtechnology/ros-foxy-orbslam2:1.0.0"
    )
    local image
    for image in "${preferred[@]}"; do
        if docker image inspect "$image" >/dev/null 2>&1; then
            echo "$image"
            return 0
        fi
    done
    return 1
}

collect_device_args() {
    local candidates=(
        /dev/astradepth
        /dev/astrauvc
        /dev/video0
        /dev/video1
        /dev/myserial
        /dev/ydlidar
        /dev/rplidar
        /dev/input
    )
    local dev
    for dev in "${candidates[@]}"; do
        if [[ -e "$dev" ]]; then
            echo "$dev"
        fi
    done
}

if ! DOCKER_IMAGE="$(resolve_docker_image)"; then
    echo "ERROR: No ROS2 Docker image found."
    echo "Install one of:"
    echo "  docker pull yahboomtechnology/ros-foxy:4.0.7R2"
    echo "  (or ensure yahboomtechnology/ros-foxy-orbslam2:1.0.0 exists)"
    exit 1
fi