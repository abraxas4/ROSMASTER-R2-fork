#!/bin/bash
# Shared RTAB-Map mapping startup (camera + managed launch).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

MODE="${1:?mode required: continue|new}"
CAMERA_WAIT_SEC="${CAMERA_WAIT_SEC:-25}"
CAMERA_LOG="${CAMERA_LOG:-/tmp/rtabmap_camera.log}"
MANAGED_LAUNCH="$SCRIPT_DIR/launch/rtabmap_managed.launch.py"

echo "=== R2 RTAB-Map mapping (${MODE}) ==="

bash "$SCRIPT_DIR/stop_robot_stack.sh"

if ! ls /dev/AstraPlus /dev/AstraPlus_rgb /dev/ydlidar /dev/myserial 2>/dev/null | head -1 >/dev/null; then
  echo "ERROR: camera or robot devices missing."
  exit 1
fi

REG_OUT="$(python3 "$SCRIPT_DIR/map_registry.py" "$MODE")"
eval "$(echo "$REG_OUT" | grep -E '^(ACTIVE_|NEW_|DELETE_DB_ON_START)')"

DB_PATH="${ACTIVE_MAP_DB:-${NEW_MAP_DB:-$HOME/.ros/rtabmap.db}}"
MAP_LABEL="${ACTIVE_MAP_NAME:-${NEW_MAP_ID:-unknown}}"
DELETE_DB_LAUNCH="delete_db:=false"
if [[ "${DELETE_DB_ON_START:-0}" == "1" ]]; then
  DELETE_DB_LAUNCH="delete_db:=true"
fi

echo "Map: ${MAP_LABEL}"
echo "DB:  ${DB_PATH}"
if [[ "${DELETE_DB_ON_START:-0}" == "1" ]]; then
  echo "Mode: NEW (database reset on start)"
else
  echo "Mode: CONTINUE (append to existing database)"
fi
echo ""

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
  tail -20 "${CAMERA_LOG}" || true
  exit 1
fi
echo "Camera ready."

echo "[2/2] Starting RTAB-Map..."
echo "Rename later: python3 $SCRIPT_DIR/map_registry.py rename '집'"
echo ""

ros2 launch "$MANAGED_LAUNCH" \
  localization:=false \
  database_path:="${DB_PATH}" \
  "${DELETE_DB_LAUNCH}"