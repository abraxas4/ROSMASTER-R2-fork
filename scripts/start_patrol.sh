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

if [[ ! -f "$EXPORT_YAML" ]]; then
  echo "[2b/4] Exporting 2D map for Nav2 (from /map topic)..."
  if ! bash "$SCRIPT_DIR/export_rtabmap_map.sh"; then
    echo "ERROR: map export failed. See /tmp/patrol_loc.log"
    tail -20 /tmp/patrol_loc.log 2>/dev/null || true
    exit 1
  fi
fi

MAP_YAML="$(python3 -c "
import json, pathlib
reg=json.loads(pathlib.Path('${HOME}/maps/registry.json').read_text())
print(reg['maps']['${ACTIVE_MAP_ID}'].get('export_yaml','${EXPORT_YAML}'))
")"

if [[ ! -f "$MAP_YAML" ]]; then
  echo "ERROR: Nav2 map yaml not found: ${MAP_YAML}"
  exit 1
fi

wait_for_lifecycle_active() {
  local node="$1"
  local timeout="${2:-90}"
  local elapsed=0
  while (( elapsed < timeout )); do
    local state
    state="$(ros2 lifecycle get "${node}" 2>/dev/null | awk '{print $1}')"
    if [[ "${state}" == "active" ]]; then
      echo "  ${node}: active"
      return 0
    fi
    sleep 2
    elapsed=$((elapsed + 2))
  done
  echo "ERROR: ${node} not active after ${timeout}s (state=${state:-unknown})"
  return 1
}

echo "[3/4] Nav2 (${MAP_YAML})..."
ros2 launch nav2_bringup bringup_launch.py \
  map:="${MAP_YAML}" \
  params_file:="${NAV_PARAMS}" \
  use_sim_time:=false > /tmp/patrol_nav2.log 2>&1 &
NAV_PID=$!
echo "Waiting for map_server + AMCL + Nav2 (up to 120s)..."
if ! wait_for_lifecycle_active /map_server 120; then
  tail -20 /tmp/patrol_nav2.log 2>/dev/null || true
  exit 1
fi
if ! wait_for_lifecycle_active /amcl 60; then
  tail -20 /tmp/patrol_nav2.log 2>/dev/null || true
  exit 1
fi
if ! wait_for_lifecycle_active /bt_navigator 60; then
  tail -20 /tmp/patrol_nav2.log 2>/dev/null || true
  exit 1
fi

echo "[3b/4] Seeding AMCL from RTAB-Map pose..."
if ! python3 "$SCRIPT_DIR/publish_initial_pose.py"; then
  echo "WARN: initial pose publish failed — patrol may drift"
fi
sleep 2

echo "[4/4] Geofence patrol loop..."
echo "  - Geofence: mapped area minus margin (see ~/maps/geofence/)"
echo "  - Waypoints: known-free cells only (not map unknown edges)"
echo "  - Obstacles: lidar via Nav2 DWB planner"
echo "  - Stop: Ctrl+C or R2 순찰 중지"
echo ""

python3 "$SCRIPT_DIR/geofence_patrol.py" --loop --margin 1.0 --waypoint-inset 0.8 &
PATROL_PID=$!

wait "$PATROL_PID"