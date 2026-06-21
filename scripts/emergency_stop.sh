#!/bin/bash
# Emergency: kill zombie ROS nodes and try to silence the motor board.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== EMERGENCY STOP ==="

pkill -9 -f 'ros2 launch' 2>/dev/null || true
pkill -9 -f 'yahboomcar_base_node/base_node' 2>/dev/null || true
pkill -9 -f 'yahboomcar_bringup' 2>/dev/null || true
pkill -9 -f 'yahboomcar_ctrl' 2>/dev/null || true
pkill -9 -x joy_node 2>/dev/null || true
pkill -9 -x imu_filter_madgwick_node 2>/dev/null || true
pkill -9 -f ydlidar 2>/dev/null || true
pkill -9 -f astra_camera 2>/dev/null || true
pkill -9 -f rtabmap 2>/dev/null || true
pkill -9 -f slam_gmapping 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f joint_state_publisher 2>/dev/null || true
pkill -9 -f robot_localization 2>/dev/null || true

sleep 2

python3 - <<'PY' || true
import time
from Rosmaster_Lib import Rosmaster

for i in range(5):
    try:
        car = Rosmaster()
        time.sleep(0.3)
        car.set_beep(0)
        car.set_car_motion(0, 0, 0)
        car.reset_car_state()
        time.sleep(0.1)
        car.set_beep(0)
        v = car.get_battery_voltage()
        ver = car.get_version()
        print(f"attempt {i+1}: battery={v}, version={ver}")
        del car
        if v > 0 and ver > 0:
            break
    except Exception as exc:
        print(f"attempt {i+1}: {exc}")
    time.sleep(0.3)
PY

echo ""
echo "If beeping continues and battery=0.0 / version=-1:"
echo "  1. Power OFF the robot main switch (not just Ctrl+C)"
echo "  2. Wait 10 seconds"
echo "  3. Power ON, wait 5s for gyro init"
echo "  4. Run: bash $SCRIPT_DIR/emergency_stop.sh"