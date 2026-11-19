#!/bin/bash
# Patrol on the active Cartographer map.
# PATROL_UPDATE_MAP=1 (default): Cartographer stays on and the map is updated.
# PATROL_UPDATE_MAP=0: frozen yaml/pgm + AMCL, map does not change.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

PATROL_UPDATE_MAP="${PATROL_UPDATE_MAP:-1}"
NAV_PARAMS="${NAV_PARAMS:-$SCRIPT_DIR/params/r2_nav.yaml}"

echo "=== R2 Geofence Patrol ==="
echo "PATROL_UPDATE_MAP=${PATROL_UPDATE_MAP}  (1=update map while driving, 0=frozen AMCL map)"

bash "$SCRIPT_DIR/stop_robot_stack.sh"

if ! python3 "$SCRIPT_DIR/map_registry.py" continue >/tmp/patrol_registry.env 2>/tmp/patrol_registry.err; then
  echo "No registered Nav2 map. Converting latest Cartographer pbstream..."
  bash "$SCRIPT_DIR/save_cartographer_map.sh"
  python3 "$SCRIPT_DIR/map_registry.py" continue >/tmp/patrol_registry.env
fi
# shellcheck disable=SC1091
source /tmp/patrol_registry.env

MAP_YAML="${ACTIVE_MAP_YAML:?}"
MAP_LABEL="${ACTIVE_MAP_NAME:-${ACTIVE_MAP_ID:-map}}"
MAP_PBSTREAM="${ACTIVE_MAP_PBSTREAM:-}"

if [[ ! -f "$MAP_YAML" ]]; then
  echo "오류: Nav2 맵 yaml이 없습니다: ${MAP_YAML}"
  exit 1
fi
if [[ ! -f "$NAV_PARAMS" ]]; then
  echo "오류: Nav2 설정 파일이 없습니다: ${NAV_PARAMS}"
  exit 1
fi

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"
set -u

touch /tmp/r2_serial_for_driver
sleep 0.4

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

wait_for_lifecycle_active() {
  local node="$1"
  local timeout="${2:-90}"
  local elapsed=0
  local state=""
  while (( elapsed < timeout )); do
    state="$(timeout 4 ros2 lifecycle get "${node}" 2>/dev/null \
      | grep -Eo 'unconfigured|inactive|active|finalized' | tail -1 || true)"
    if [[ "${state}" == "active" ]]; then
      echo "  ${node}: active"
      return 0
    fi
    if grep -q 'Managed nodes are active' /tmp/patrol_nav2.log 2>/dev/null; then
      echo "  ${node}: active (nav2 log)"
      return 0
    fi
    if grep -q 'Failed to bring up all requested nodes' /tmp/patrol_nav2.log 2>/dev/null; then
      echo "오류: Nav2가 켜지다 실패했습니다. /tmp/patrol_nav2.log 를 보세요."
      return 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
  done
  echo "오류: ${node} 가 ${timeout}초 안에 켜지지 않았습니다 (state=${state:-unknown})"
  return 1
}

LOC_PID=""
NAV_PID=""
PATROL_PID=""

cleanup() {
  echo ""
  echo "Stopping patrol stack..."
  kill "$LOC_PID" "$NAV_PID" "$PATROL_PID" 2>/dev/null || true
  wait "$LOC_PID" "$NAV_PID" "$PATROL_PID" 2>/dev/null || true
  bash "$SCRIPT_DIR/stop_robot_stack.sh"
}
trap cleanup EXIT INT TERM

if [[ "${PATROL_UPDATE_MAP}" == "1" ]]; then
  if [[ -z "$MAP_PBSTREAM" || ! -f "$MAP_PBSTREAM" ]]; then
    echo "오류: Cartographer pbstream이 없습니다: ${MAP_PBSTREAM}"
    exit 1
  fi

  echo "[1/4] Cartographer 맵 불러오기 (${MAP_LABEL})..."
  echo "  pbstream: ${MAP_PBSTREAM}"
  python3 "$SCRIPT_DIR/r2_status.py" "맵에서 지금 위치를 찾는 중입니다…"
  ros2 launch "$SCRIPT_DIR/launch/cartographer_managed.launch.py" \
    load_state_filename:="${MAP_PBSTREAM}" \
    load_frozen_state:=false \
    start_trajectory_with_default_topics:=false \
    > /tmp/patrol_loc.log 2>&1 &
  LOC_PID=$!
  if ! wait_for_topic /scan 25; then
    echo "오류: 라이다(/scan)가 안 올라왔습니다. /tmp/patrol_loc.log 를 보세요."
    python3 "$SCRIPT_DIR/r2_status.py" "라이다가 안 켜졌습니다."
    tail -20 /tmp/patrol_loc.log 2>/dev/null || true
    exit 1
  fi
  if ! wait_for_topic /map 40; then
    echo "오류: 맵(/map)이 안 올라왔습니다. /tmp/patrol_loc.log 를 보세요."
    python3 "$SCRIPT_DIR/r2_status.py" "맵을 불러오지 못했습니다."
    tail -20 /tmp/patrol_loc.log 2>/dev/null || true
    exit 1
  fi

  echo "[2/4] 지금 자리가 맵의 어디인지 맞추는 중..."
  set +e
  python3 "$SCRIPT_DIR/relocalize.py" --map-yaml "${MAP_YAML}" --min-score 0.42
  RELOC=$?
  set -e
  if [[ "$RELOC" -ne 0 ]]; then
    echo "조금 돌아다녀봤지만 모르는 맵입니다. 순찰을 시작하지 않습니다."
    python3 "$SCRIPT_DIR/r2_status.py" "조금 돌아다녀봤지만 모르는 맵입니다."
    exit 1
  fi
  if ! wait_for_topic /odom 20; then
    echo "오류: 위치(/odom)가 안 나옵니다."
    python3 "$SCRIPT_DIR/r2_status.py" "위치를 찾았지만 오돔이 없습니다."
    exit 1
  fi

  echo "[3/4] Nav2 길 계획..."
  ros2 launch nav2_bringup navigation_launch.py \
    params_file:="${NAV_PARAMS}" \
    use_sim_time:=false \
    use_composition:=False \
    autostart:=true > /tmp/patrol_nav2.log 2>&1 &
  NAV_PID=$!
  echo "Nav2를 기다리는 중 (최대 120초)..."
  if ! wait_for_lifecycle_active /bt_navigator 120; then
    echo "오류: Nav2가 활성화되지 않았습니다. /tmp/patrol_nav2.log 를 보세요."
    python3 "$SCRIPT_DIR/r2_status.py" "Nav2가 켜지지 않았습니다."
    tail -20 /tmp/patrol_nav2.log 2>/dev/null || true
    exit 1
  fi

  python3 "$SCRIPT_DIR/r2_status.py" ""
  echo "[4/4] 순찰 시작 (아는 곳 + 새로 열린 맵)"
  echo "  중지: R2 순찰 중지"
  echo ""

  python3 "$SCRIPT_DIR/geofence_patrol.py" \
    --loop --margin 1.0 --waypoint-inset 0.8 \
    --localizer bt_navigator --explore-new &
  PATROL_PID=$!
else
  echo "[1/3] Lidar + odom (${MAP_LABEL})..."
  ros2 launch "$SCRIPT_DIR/launch/patrol_managed.launch.py" > /tmp/patrol_loc.log 2>&1 &
  LOC_PID=$!
  if ! wait_for_topic /scan 25; then
    echo "오류: 라이다(/scan)가 안 올라왔습니다. /tmp/patrol_loc.log 를 보세요."
    tail -20 /tmp/patrol_loc.log 2>/dev/null || true
    exit 1
  fi
  if ! wait_for_topic /odom 20; then
    echo "오류: 오돔(/odom)이 안 나옵니다. /tmp/patrol_loc.log 를 보세요."
    tail -20 /tmp/patrol_loc.log 2>/dev/null || true
    exit 1
  fi

  echo "[2/3] Nav2 + AMCL frozen map (${MAP_YAML})..."
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

  echo "[2b/3] Seeding AMCL at map origin (start the robot where mapping started)..."
  if ! python3 "$SCRIPT_DIR/publish_initial_pose.py"; then
    echo "WARN: initial pose publish failed — patrol may not localize"
  fi
  sleep 2

  echo "[3/3] Geofence patrol loop (frozen map)..."
  echo "  - Map: ${MAP_YAML}"
  echo "  - Stop: Ctrl+C or R2 순찰 중지"
  echo ""

  python3 "$SCRIPT_DIR/geofence_patrol.py" --loop --margin 1.0 --waypoint-inset 0.8 &
  PATROL_PID=$!
fi

wait "$PATROL_PID"
