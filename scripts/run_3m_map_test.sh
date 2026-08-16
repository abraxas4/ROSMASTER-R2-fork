#!/bin/bash
# Fresh 3 m mapping test on an isolated DB, then auto-stop and write HTML.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/run_docker_env.sh"
set +u
# shellcheck source=/dev/null
source "$SCRIPT_DIR/native_ros_setup.bash"
set -u

TEST_DB="$HOME/maps/databases/drive_test_sensors.db"
LOGDIR=/tmp/r2_sensor_test
mkdir -p "$LOGDIR"

echo "=== 1. stop leftover ==="
bash "$SCRIPT_DIR/stop_by_pid.sh"

# wait USB
for i in $(seq 1 15); do
  if [[ -e /dev/ydlidar && -e /dev/AstraPlus ]]; then
    echo "sensors present"
    break
  fi
  sleep 1
done

rm -f "$TEST_DB"
echo "TEST_DB=$TEST_DB (fresh)"

echo "=== 2. camera ==="
nohup ros2 launch astra_camera astro_pro_plus.launch.xml >"$LOGDIR/camera.log" 2>&1 &
echo $! >"$LOGDIR/camera.pid"
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qx /camera/color/image_raw; then
    echo "camera up ${i}s"
    break
  fi
  sleep 1
done

echo "=== 3. mapping ==="
nohup ros2 launch "$SCRIPT_DIR/launch/rtabmap_managed.launch.py" \
  database_path:="${TEST_DB}" \
  delete_db:=true \
  >"$LOGDIR/rtabmap.log" 2>&1 &
echo $! >"$LOGDIR/rtabmap.pid"

for i in $(seq 1 30); do
  ok=0
  list=$(ros2 topic list 2>/dev/null || true)
  echo "$list" | grep -qx /scan && ok=$((ok+1))
  echo "$list" | grep -qx /odom && ok=$((ok+1))
  echo "$list" | grep -qx /rgbd_image && ok=$((ok+1))
  if [[ $ok -ge 3 ]]; then
    echo "topics up ${i}s"
    break
  fi
  sleep 1
done

sleep 2
echo "=== 3b. raw samples ==="
set +e
python3 "$SCRIPT_DIR/dump_sensor_samples.py"
set -e

echo "=== 4. drive ==="
set +e
python3 "$SCRIPT_DIR/drive_3m.py"
DRIVE_RC=$?
set -e
echo "drive rc=$DRIVE_RC"
sleep 3

echo "=== 4b. occupancy map image ==="
set +e
python3 "$SCRIPT_DIR/dump_sensor_samples.py" --map-only
set -e

echo "=== 5. report ==="
python3 "$SCRIPT_DIR/report_drive_test.py"
python3 "$SCRIPT_DIR/write_full_map_report.py"

echo "=== 6. auto stop ==="
bash "$SCRIPT_DIR/stop_by_pid.sh"
echo "ALL_OFF"
exit "$DRIVE_RC"
