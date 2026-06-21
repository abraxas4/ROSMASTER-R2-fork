#!/bin/bash
# Quick ROS camera topic check for Orbbec Astra Plus on R2.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

WAIT_SEC=15

pass() { echo "[PASS] $*"; }
fail() { echo "[FAIL] $*"; }

echo "=== Orbbec camera ROS diagnosis ==="

for dev in /dev/AstraPlus /dev/AstraPlus_rgb; do
  if [[ -e "$dev" ]]; then
    pass "device: $dev"
  else
    fail "missing: $dev"
    exit 1
  fi
done
if [[ -e /dev/video0 ]]; then
  pass "device: /dev/video0 (UVC idle)"
else
  echo "[INFO] /dev/video0 not listed (normal while astra_camera holds the UVC interface)"
fi

pkill -f astra_camera_node 2>/dev/null || true
sleep 1

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

ros2 launch astra_camera astro_pro_plus.launch.xml > /tmp/r2_camera_diag.log 2>&1 &
LP=$!
sleep "${WAIT_SEC}"

FAIL=0
check_topic() {
  local topic="$1" min_hz="$2"
  if ! ros2 topic list 2>/dev/null | grep -qx "$topic"; then
    fail "topic missing: $topic"
    FAIL=$((FAIL + 1))
    return
  fi
  pass "topic exists: $topic"
  local hz
  hz=$(timeout 6 ros2 topic hz "$topic" 2>/dev/null | awk '/average rate/{print $3; exit}')
  if [[ -z "$hz" ]]; then
    fail "no messages: $topic"
    FAIL=$((FAIL + 1))
    return
  fi
  awk -v hz="$hz" -v min="$min_hz" 'BEGIN {exit (hz+0 >= min+0)?0:1}' \
    && pass "rate OK: $topic ${hz} Hz" \
    || { fail "rate low: $topic ${hz} Hz (min ${min_hz})"; FAIL=$((FAIL + 1)); }
}

check_topic /camera/color/image_raw 10
check_topic /camera/depth/image_raw 10
check_topic /camera/depth/points 5

kill "$LP" 2>/dev/null || true
wait "$LP" 2>/dev/null || true

echo ""
if [[ "$FAIL" -eq 0 ]]; then
  echo "Camera ready as ROS sensor."
  echo "Start streaming: bash $SCRIPT_DIR/start_camera.sh"
else
  echo "See log: /tmp/r2_camera_diag.log"
  exit 1
fi