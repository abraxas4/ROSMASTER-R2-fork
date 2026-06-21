#!/bin/bash
# Start RGB-D + lidar fusion SLAM (RTAB-Map) on native ROS2 Humble.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

CAMERA_WAIT_SEC="${CAMERA_WAIT_SEC:-25}"
CAMERA_LOG="${CAMERA_LOG:-/tmp/rtabmap_camera.log}"

echo "=== R2 RTAB-Map mapping (camera + 4ROS lidar) ==="
echo "library_ws: ${LIBRARY_WS_HOST}"
echo "workspace:  ${NATIVE_WS_HOST}"
echo ""

bash "$SCRIPT_DIR/stop_robot_stack.sh"

if ! ls /dev/AstraPlus /dev/AstraPlus_rgb /dev/ydlidar /dev/myserial 2>/dev/null | head -1 >/dev/null; then
  echo "ERROR: camera or robot devices missing."
  echo "Check: ls -la /dev/AstraPlus* /dev/ydlidar /dev/myserial"
  exit 1
fi

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

wait_for_topic() {
  local topic="$1"
  local timeout="$2"
  local elapsed=0
  while (( elapsed < timeout )); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

cleanup() {
  echo ""
  echo "Stopping camera and mapping nodes..."
  kill "$CAM_PID" 2>/dev/null || true
  wait "$CAM_PID" 2>/dev/null || true
  bash "$SCRIPT_DIR/stop_robot_stack.sh"
}
trap cleanup EXIT INT TERM

echo "[1/2] Starting Orbbec camera..."
ros2 launch astra_camera astro_pro_plus.launch.xml >"${CAMERA_LOG}" 2>&1 &
CAM_PID=$!

if ! wait_for_topic /camera/color/image_raw "${CAMERA_WAIT_SEC}"; then
  echo "ERROR: camera topic not ready within ${CAMERA_WAIT_SEC}s"
  echo "Log: ${CAMERA_LOG}"
  tail -20 "${CAMERA_LOG}" || true
  exit 1
fi
echo "Camera ready: /camera/color/image_raw"

echo "[2/2] Starting RTAB-Map fusion SLAM..."
echo "Move the robot slowly. Stop with Ctrl+C."
echo ""
echo "GUI on R2 monitor (separate SSH session):"
echo "  DISPLAY=:0 bash $SCRIPT_DIR/view_rtabmap_viz.sh"
echo ""

ros2 launch yahboomcar_nav map_rtabmap_launch.py