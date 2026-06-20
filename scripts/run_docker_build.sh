#!/bin/bash
# ROSMASTER R2 - Docker 안에서 colcon build (비대화형, SSH/자동화용)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker.sh
source "$SCRIPT_DIR/run_docker_env.sh"

echo "=== colcon build in Docker ==="
echo "Image: ${DOCKER_IMAGE}"
echo "Workspace: ${WORKSPACE_HOST}"

DEVICE_ARGS=()
while IFS= read -r dev; do
    [[ -n "$dev" ]] && DEVICE_ARGS+=(--device="$dev")
done < <(collect_device_args)

docker run --rm \
  --net=host \
  -v "${WORKSPACE_HOST}:/root/yahboomcar_ros2_ws" \
  -v "${USER_HOME}/temp:/root/yahboomcar_ros2_ws/temp" \
  -v "${USER_HOME}/rosboard:/root/rosboard" \
  -v "${USER_HOME}/maps:/root/maps" \
  "${DEVICE_ARGS[@]}" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -e
    source /opt/ros/foxy/setup.bash
    cd /root/yahboomcar_ros2_ws
    colcon build --symlink-install \
      --packages-skip yahboomcar_KCFTracker yahboomcar_mediapipe yahboomcar_visual yahboomcar_slam
  '

echo "Build finished."