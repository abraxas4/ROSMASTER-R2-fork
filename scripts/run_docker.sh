#!/bin/bash
# ROSMASTER R2 - Docker 실행 스크립트 (Jetson Orin용, Git 연동)
# code/yahboomcar_ros2_ws 를 컨테이너의 /root/yahboomcar_ros2_ws 로 직접 마운트

set -euo pipefail

echo "=== ROSMASTER R2 Docker 시작 (Git 관리 버전) ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

xhost +

echo "Mounting workspace: ${WORKSPACE_HOST} -> /root/yahboomcar_ros2_ws"
echo "Using image: ${DOCKER_IMAGE}"
echo ""

DEVICE_ARGS=()
while IFS= read -r dev; do
    [[ -n "$dev" ]] && DEVICE_ARGS+=(--device="$dev")
done < <(collect_device_args)

docker run -it \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${WORKSPACE_HOST}:/root/yahboomcar_ros2_ws" \
  -v "${USER_HOME}/temp:/root/yahboomcar_ros2_ws/temp" \
  -v "${USER_HOME}/rosboard:/root/rosboard" \
  -v "${USER_HOME}/maps:/root/maps" \
  "${DEVICE_ARGS[@]}" \
  -p 9090:9090 \
  -p 8888:8888 \
  "${DOCKER_IMAGE}" /bin/bash