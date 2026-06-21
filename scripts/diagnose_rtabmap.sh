#!/bin/bash
# Verify RTAB-Map fusion stack starts and sees camera + lidar topics.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

WAIT_SEC=18
PASS=0
FAIL=0
pass() { echo "[PASS] $*"; PASS=$((PASS + 1)); }
fail() { echo "[FAIL] $*"; FAIL=$((FAIL + 1)); }

echo "=== RTAB-Map fusion diagnosis ==="

bash "$SCRIPT_DIR/stop_robot_stack.sh"

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

ros2 launch astra_camera astro_pro_plus.launch.xml >/tmp/r2_rtabmap_cam.log 2>&1 &
CP=$!

cam_wait=0
while (( cam_wait < 25 )); do
  if ros2 topic list 2>/dev/null | grep -qx /camera/color/image_raw; then
    break
  fi
  sleep 1
  cam_wait=$((cam_wait + 1))
done

ros2 launch yahboomcar_nav map_rtabmap_launch.py >/tmp/r2_rtabmap_map.log 2>&1 &
MP=$!
sleep "${WAIT_SEC}"

check_hz() {
  local topic="$1" min="$2"
  if ! ros2 topic list 2>/dev/null | grep -qx "$topic"; then
    fail "topic missing: $topic"
    return
  fi
  pass "topic exists: $topic"
  local hz
  hz=$(timeout 5 ros2 topic hz "$topic" 2>/dev/null | awk '/average rate/{print $3; exit}' || true)
  if [[ -z "$hz" ]]; then
    if timeout 4 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      pass "messages OK: $topic"
      return
    fi
    fail "no messages: $topic"
    return
  fi
  awk -v hz="$hz" -v min="$min" 'BEGIN {exit (hz+0 >= min+0)?0:1}' \
    && pass "rate OK: $topic ${hz} Hz" \
    || fail "rate low: $topic ${hz} Hz (min ${min})"
}

check_hz /camera/color/image_raw 10
check_hz /camera/depth/image_raw 3
check_hz /scan 3
check_hz /odom 3

if pgrep -af 'rtabmap_slam.*/rtabmap' >/dev/null 2>&1; then
  pass "rtabmap node running"
else
  fail "rtabmap node not running"
  tail -10 /tmp/r2_rtabmap_map.log 2>/dev/null || true
fi

kill "$CP" "$MP" 2>/dev/null || true
wait "$CP" "$MP" 2>/dev/null || true
bash "$SCRIPT_DIR/stop_robot_stack.sh" >/dev/null

echo ""
echo "Result: PASS=${PASS} FAIL=${FAIL}"
[[ "$FAIL" -eq 0 ]]