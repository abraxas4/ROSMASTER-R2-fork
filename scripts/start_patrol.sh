#!/bin/bash
# Start localization + Nav2 + geofence patrol on the active map.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

CAMERA_WAIT_SEC="${CAMERA_WAIT_SEC:-25}"
CAMERA_LOG="${CAMERA_LOG:-/tmp/patrol_camera.log}"
LOC_LAUNCH="$SCRIPT_DIR/launch/rtabmap_localization_managed.launch.py"
NAV_PARAMS="${NATIVE_WS_HOST}/install/yahboomcar_nav/share/yahboomcar_nav/params/rtabmap_nav_params.yaml"

echo "=== R2 Geofence Patrol ==="

bash "$SCRIPT_DIR/stop_robot_stack.sh"

REG_OUT="$(python3 "$SCRIPT_DIR/map_registry.py" continue)"
eval "$(echo "$REG_OUT" | grep -E '^ACTIVE_MAP_')"

DB_PATH="${ACTIVE_MAP_DB:?}"
MAP_LABEL="${ACTIVE_MAP_NAME:-${ACTIVE_MAP_ID:-map}}"

if [[ ! -f "$DB_PATH" ]]; then
  echo "ERROR: no map database. Run mapping first."
  exit 1
fi

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

EXPORT_YAML="${HOME}/maps/exported/${ACTIVE_MAP_ID}/nav_map.yaml"
if [[ ! -f "$EXPORT_YAML" ]]; then
  echo "Exporting 2D map for Nav2..."
  bash "$SCRIPT_DIR/export_rtabmap_map.sh"
fi
MAP_YAML="$(python3 -c "
import json, pathlib
reg=json.loads(pathlib.Path('${HOME}/maps/registry.json').read_text())
print(reg['maps']['${ACTIVE_MAP_ID}'].get('export_yaml','${EXPORT_YAML}'))
")"

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
  echo "Stopping patrol stack..."
  kill "$CAM_PID" "$LOC_PID" "$NAV_PID" "$PATROL_PID" 2>/dev/null || true
  wait "$CAM_PID" "$LOC_PID" "$NAV_PID" "$PATROL_PID" 2>/dev/null || true
  bash "$SCRIPT_DIR/stop_robot_stack.sh"
}
trap cleanup EXIT INT TERM

echo "[1/4] Camera..."
ros2 launch astra_camera astro_pro_plus.launch.xml >"${CAMERA_LOG}" 2>&1 &
CAM_PID=$!
if ! wait_for_topic /camera/color/image_raw "${CAMERA_WAIT_SEC}"; then
  echo "ERROR: camera not ready"
  exit 1
fi

echo "[2/4] RTAB-Map localization (${MAP_LABEL})..."
ros2 launch "$LOC_LAUNCH" database_path:="${DB_PATH}" > /tmp/patrol_loc.log 2>&1 &
LOC_PID=$!
sleep 15

echo "[3/4] Nav2 (${MAP_YAML})..."
ros2 launch nav2_bringup bringup_launch.py \
  map:="${MAP_YAML}" \
  params_file:="${NAV_PARAMS}" \
  use_sim_time:=false > /tmp/patrol_nav2.log 2>&1 &
NAV_PID=$!
echo "Waiting for Nav2 (up to 90s)..."
sleep 30

echo "[4/4] Geofence patrol loop..."
echo "  - Geofence: mapped area minus margin (see ~/maps/geofence/)"
echo "  - Obstacles: lidar via Nav2 DWB planner"
echo "  - Stop: Ctrl+C or R2 매핑 중지"
echo ""

python3 "$SCRIPT_DIR/geofence_patrol.py" --loop &
PATROL_PID=$!

wait "$PATROL_PID"