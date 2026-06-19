#!/bin/bash
# ROSMASTER R2 - Docker 실행 스크립트 (Jetson Orin용, Git 연동)
# code/yahboomcar_ros2_ws 를 컨테이너의 /root/yahboomcar_ros2_ws 로 직접 마운트

echo "=== ROSMASTER R2 Docker 시작 (Git 관리 버전) ==="

xhost +

# === 사용자 환경에 맞게 수정하세요 ===
GIT_REPO_PATH="/home/jetson/ROSMASTER-R2-fork"
WORKSPACE_HOST="${GIT_REPO_PATH}/code/yahboomcar_ros2_ws"

# Docker 이미지 태그 (실제 사용하는 최신 버전으로 변경)
DOCKER_IMAGE="yahboomtechnology/ros-foxy:3.5.4"

echo "Mounting workspace: ${WORKSPACE_HOST} -> /root/yahboomcar_ros2_ws"
echo "Using image: ${DOCKER_IMAGE}"
echo ""

docker run -it \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${WORKSPACE_HOST}:/root/yahboomcar_ros2_ws" \
  -v /home/jetson/temp:/root/yahboomcar_ros2_ws/temp \
  -v /home/jetson/rosboard:/root/rosboard \
  -v /home/jetson/maps:/root/maps \
  --device=/dev/astradepth \
  --device=/dev/astrauvc \
  --device=/dev/video0 \
  --device=/dev/myserial \
  --device=/dev/rplidar \
  --device=/dev/input \
  -p 9090:9090 \
  -p 8888:8888 \
  ${DOCKER_IMAGE} /bin/bash
