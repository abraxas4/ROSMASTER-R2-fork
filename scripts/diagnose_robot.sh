#!/bin/bash
# R2 pre-mapping diagnosis: devices, workspace, optional ROS topic checks.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

RUN_ROS=false
RUN_MOTION=false
ROS_WAIT_SEC=18

usage() {
  cat <<'EOF'
Usage: bash scripts/diagnose_robot.sh [options]

Options:
  --ros       Start bringup + 4ROS lidar in Docker and verify ROS topics
  --motion    With --ros, send a tiny cmd_vel pulse (robot moves slightly)
  -h, --help  Show this help

Examples:
  bash scripts/diagnose_robot.sh
  bash scripts/diagnose_robot.sh --ros
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros) RUN_ROS=true ;;
    --motion) RUN_MOTION=true; RUN_ROS=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
  shift
done

PASS=0
WARN=0
FAIL=0

pass() { echo "[PASS] $*"; PASS=$((PASS + 1)); }
warn() { echo "[WARN] $*"; WARN=$((WARN + 1)); }
fail() { echo "[FAIL] $*"; FAIL=$((FAIL + 1)); }

check_file() {
  local path="$1" label="$2"
  if [[ -e "$path" ]]; then
    pass "$label: $path"
  else
    fail "$label missing: $path"
  fi
}

echo "=== R2 Robot Diagnosis ==="
echo "Time: $(date)"
echo ""

echo "--- Phase 1: Host / hardware / workspace ---"

check_file "$GIT_REPO_PATH" "fork repo"
check_file "$WORKSPACE_HOST/install/setup.bash" "fork workspace install"
check_file "$LIBRARY_WS_HOST/install/setup.bash" "library_ws install"
check_file "$LIBRARY_WS_HOST/install/slam_gmapping" "slam_gmapping package"
check_file "$LIBRARY_WS_HOST/install/ydlidar_ros2_driver" "ydlidar driver package"

if docker image inspect "$DOCKER_IMAGE" >/dev/null 2>&1; then
  pass "Docker image: $DOCKER_IMAGE"
else
  fail "Docker image missing: $DOCKER_IMAGE"
fi

for dev in /dev/myserial /dev/ydlidar; do
  if [[ -e "$dev" ]]; then
    pass "device present: $dev -> $(readlink -f "$dev" 2>/dev/null || echo '?')"
  else
    fail "device missing: $dev"
  fi
done

if compgen -G "/dev/ttyUSB*" >/dev/null; then
  pass "USB serial nodes: $(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"
else
  warn "no /dev/ttyUSB* (power on robot and wait a few seconds)"
fi

if [[ -f "$HOME/.config/autostart/start_app.sh.desktop" ]]; then
  warn "Yahboom phone app autostart is enabled (may grab /dev/myserial)"
elif [[ -f "$HOME/.config/autostart/start_app.sh.desktop.disabled" ]]; then
  pass "Yahboom phone app autostart disabled"
else
  warn "Yahboom autostart entry not found"
fi

if pgrep -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' >/dev/null 2>&1; then
  warn "rosmaster_main.py is running (blocks /dev/myserial for ROS driver)"
else
  pass "rosmaster_main.py not running"
fi

if [[ -e /dev/astradepth || -e /dev/astrauvc ]]; then
  pass "depth camera device detected (optional for lidar-only mapping)"
else
  warn "no Astra camera device (OK for lidar-only mapping)"
fi

echo ""
echo "--- Phase 1 result: PASS=$PASS WARN=$WARN FAIL=$FAIL ---"

if [[ "$RUN_ROS" != true ]]; then
  echo ""
  echo "Next: run ROS topic checks"
  echo "  bash $SCRIPT_DIR/diagnose_robot.sh --ros"
  echo ""
  echo "If Phase 1 has FAIL items, fix those before mapping."
  [[ "$FAIL" -eq 0 ]]
  exit $?
fi

if [[ "$FAIL" -gt 0 ]]; then
  echo ""
  fail "aborting ROS checks because Phase 1 has failures"
  exit 1
fi

if pgrep -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' >/dev/null 2>&1; then
  echo "Stopping rosmaster_main.py before ROS checks..."
  pkill -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' || true
  sleep 1
fi

echo "Stopping stale bringup/lidar processes from earlier runs..."
pkill -f 'yahboomcar_bringup_R2_launch.py' 2>/dev/null || true
pkill -f 'ydlidar_raw_launch.py' 2>/dev/null || true
pkill -f 'Ackman_driver_R2' 2>/dev/null || true
pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
sleep 2

echo ""
echo "--- Phase 2: ROS bringup + sensor topics (native Humble) ---"
echo "Note: library_ws lidar/SLAM binaries require host ROS (not Foxy Docker)."
echo "Waiting ${ROS_WAIT_SEC}s for nodes to publish..."

ROS_PASS=0
ROS_FAIL=0
ros_pass() { echo "[PASS] $*"; ROS_PASS=$((ROS_PASS + 1)); }
ros_fail() { echo "[FAIL] $*"; ROS_FAIL=$((ROS_FAIL + 1)); }

set +e
set +u
DIAG_OUTPUT="$(
  export LIBRARY_WS_HOST NATIVE_WS_HOST RUN_MOTION ROS_WAIT_SEC
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
  export ROBOT_TYPE="${ROBOT_TYPE:-r2}"
  export RPLIDAR_TYPE="${RPLIDAR_TYPE:-4ROS}"
  export CAMERA_TYPE="${CAMERA_TYPE:-astraplus}"
  set +u
  # shellcheck source=native_ros_setup.bash
  source "$SCRIPT_DIR/native_ros_setup.bash"

  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py > /tmp/r2_diag_bringup.log 2>&1 &
  BRINGUP_PID=$!
  ros2 launch ydlidar_ros2_driver ydlidar_raw_launch.py > /tmp/r2_diag_lidar.log 2>&1 &
  LIDAR_PID=$!

  sleep "${ROS_WAIT_SEC}"

  check_hz() {
    local topic="$1" min_hz="$2"
    local line
    line=$(timeout 6 ros2 topic hz "$topic" 2>/dev/null | awk '/average rate/{print $3; exit}' || true)
    if [[ -n "$line" ]]; then
      awk -v hz="$line" -v min="$min_hz" 'BEGIN {exit (hz+0 >= min+0)?0:1}' \
        && echo "HZ_OK $topic $line" || echo "HZ_LOW $topic $line"
    else
      echo "HZ_NONE $topic"
    fi
  }

  check_hz /scan 3
  check_hz /imu/data_raw 10
  check_hz /vel_raw 10
  check_hz /odom_raw 5

  for topic in /scan /imu/data_raw /vel_raw /odom_raw; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "TOPIC_OK $topic"
    else
      echo "TOPIC_MISSING $topic"
    fi
  done

  if [[ "${RUN_MOTION}" == "true" ]]; then
    echo "MOTION_TEST start"
    timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
    sleep 1
    timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
    echo "MOTION_TEST done"
  fi

  kill "$BRINGUP_PID" "$LIDAR_PID" 2>/dev/null || true
  wait "$BRINGUP_PID" "$LIDAR_PID" 2>/dev/null || true
)"
NATIVE_RC=$?
set -u
set -e

echo "$DIAG_OUTPUT"

while IFS= read -r line; do
  case "$line" in
    TOPIC_OK*) ros_pass "topic exists: ${line#TOPIC_OK }" ;;
    TOPIC_MISSING*) ros_fail "topic missing: ${line#TOPIC_MISSING }" ;;
    HZ_OK*) ros_pass "publish rate OK: ${line#HZ_OK } Hz" ;;
    HZ_LOW*) ros_fail "publish rate low: ${line#HZ_LOW } Hz" ;;
    HZ_NONE*) ros_fail "no messages on: ${line#HZ_NONE }" ;;
    MOTION_TEST*) echo "[INFO] ${line}" ;;
  esac
done <<< "$DIAG_OUTPUT"

if [[ "$NATIVE_RC" -ne 0 ]]; then
  ros_fail "native ROS diagnosis exited with code $NATIVE_RC"
fi

echo ""
echo "--- Phase 2 result: PASS=$ROS_PASS FAIL=$ROS_FAIL ---"
echo ""
if [[ "$ROS_FAIL" -eq 0 ]]; then
  echo "All checks passed. Ready for mapping:"
  echo "  bash $SCRIPT_DIR/start_mapping.sh"
else
  echo "Fix failures above before mapping."
fi

[[ "$FAIL" -eq 0 && "$ROS_FAIL" -eq 0 ]]