#!/bin/bash
# Start 4ROS lidar SLAM mapping inside Docker (non-interactive launch).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

echo "=== R2 Mapping (4ROS) via Docker ==="
echo "Image: ${DOCKER_IMAGE}"
echo "Workspace: ${WORKSPACE_HOST}"
echo "Library: ${LIBRARY_WS_HOST}"

DEVICE_ARGS=()
while IFS= read -r dev; do
    [[ -n "$dev" ]] && DEVICE_ARGS+=(--device="$dev")
done < <(collect_device_args)

docker run --rm \
  --net=host \
  -v "${WORKSPACE_HOST}:/root/yahboomcar_ros2_ws" \
  -v "${LIBRARY_WS_HOST}/install:/root/library_ws/install" \
  -v "${SCRIPT_DIR}/docker_ros_setup.bash:/root/docker_ros_setup.bash:ro" \
  -v "${USER_HOME}/maps:/root/maps" \
  "${DEVICE_ARGS[@]}" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    source /root/docker_ros_setup.bash
    ros2 launch yahboomcar_nav map_gmapping_4ros_launch.py
  '