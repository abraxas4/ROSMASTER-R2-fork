#!/bin/bash
# Stop robot-related PIDs without pkill -f (avoids killing the caller).
set +e
# comm is often truncated to 15 chars; match by prefix, not exact -x.
names='rtabmap rtabmap_viz astra_camera ydlidar Ackman imu_filter ekf_node joy_node yahboom_joy rgbd_sync base_node_R2 robot_state_pub joint_state_pub'
for n in $names; do
  pids=$(pgrep "$n" 2>/dev/null)
  [ -n "$pids" ] && kill $pids 2>/dev/null
done
# launch wrappers: match comm python3 with specific args via pgrep -f is risky; use pgrep -x python3 + /proc
sleep 1
for n in $names; do
  pids=$(pgrep "$n" 2>/dev/null)
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
done
# leftover launch parents
for pid in $(pgrep -x python3); do
  args=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null || true)
  case "$args" in
    *rtabmap_managed.launch.py*|*rtabmap_viz_managed*|*astro_pro_plus.launch.xml*|*yahboomcar_bringup_R2_launch.py*|*ydlidar_raw_launch.py*)
      kill "$pid" 2>/dev/null
      ;;
  esac
done
sleep 0.5
python3 - <<'PY'
import time
try:
    from Rosmaster_Lib import Rosmaster
    car = Rosmaster()
    time.sleep(0.2)
    for _ in range(3):
        car.set_beep(0)
        car.set_car_motion(0, 0, 0)
    car.reset_car_state()
except Exception:
    pass
PY
echo "stop_by_pid done"
