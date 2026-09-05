#!/bin/bash
# Stop ROS nodes for mapping / patrol. Saves Cartographer state first if it is running.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Stopping robot ROS stack..."

if pgrep -x cartographer_node >/dev/null 2>&1; then
  echo "Cartographer is running — saving map before stop..."
  bash "$SCRIPT_DIR/save_cartographer_map.sh" || echo "WARN: map save failed; continuing stop"
fi

pkill -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' 2>/dev/null || true
pkill -f 'yahboomcar_bringup_R2_launch.py' 2>/dev/null || true
pkill -f 'yahboomcar_bringup_X3_launch.py' 2>/dev/null || true
pkill -f 'laser_bringup_launch.py' 2>/dev/null || true
pkill -f 'laser_bringup_no_odom_launch.py' 2>/dev/null || true
pkill -f 'bringup_no_odom_launch.py' 2>/dev/null || true
pkill -f 'ydlidar_raw_launch.py' 2>/dev/null || true
pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
pkill -f 'yahboomcar_bringup/Ackman_driver' 2>/dev/null || true
pkill -f 'yahboomcar_bringup/Mcnamu_driver' 2>/dev/null || true
pkill -f 'yahboomcar_ctrl/yahboom_joy' 2>/dev/null || true
pkill -x joy_node 2>/dev/null || true
pkill -x imu_filter_madgwick_node 2>/dev/null || true
pkill -f 'base_node_R2' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'joint_state_publisher' 2>/dev/null || true
pkill -f 'astra_camera_node' 2>/dev/null || true
pkill -f 'astro_pro_plus.launch.xml' 2>/dev/null || true
pkill -f 'map_gmapping_4ros_launch.py' 2>/dev/null || true
pkill -f 'slam_gmapping' 2>/dev/null || true
pkill -f 'robot_localization/ekf_node' 2>/dev/null || true
pkill -f 'static_transform_publisher' 2>/dev/null || true
pkill -f 'geofence_patrol.py' 2>/dev/null || true
pkill -f 'cartographer_managed.launch.py' 2>/dev/null || true
pkill -f 'patrol_managed.launch.py' 2>/dev/null || true
pkill -f 'map_cartographer_launch.py' 2>/dev/null || true
pkill -x cartographer_node 2>/dev/null || true
pkill -x cartographer_occupancy_grid_node 2>/dev/null || true
pkill -f 'cartographer_ros/cartographer_node' 2>/dev/null || true
pkill -f 'cartographer_occupancy_grid_node' 2>/dev/null || true
pkill -f 'scripts/mapping_speed_guard.py' 2>/dev/null || true
pkill -f 'scripts/log_ackermann_calib.py' 2>/dev/null || true
pkill -f 'nav2_bringup' 2>/dev/null || true
pkill -f 'controller_server' 2>/dev/null || true
pkill -f 'planner_server' 2>/dev/null || true
pkill -f 'bt_navigator' 2>/dev/null || true
pkill -f 'waypoint_follower' 2>/dev/null || true
pkill -f 'lifecycle_manager' 2>/dev/null || true
pkill -f 'map_server' 2>/dev/null || true
pkill -f 'amcl' 2>/dev/null || true

# Stop motors and silence board beeper.
python3 - <<'PY' 2>/dev/null || true
import time
from Rosmaster_Lib import Rosmaster
car = Rosmaster()
time.sleep(0.2)
for _ in range(5):
    car.set_beep(0)
car.set_car_motion(0, 0, 0)
car.reset_car_state()
time.sleep(0.1)
car.set_beep(0)
PY

sleep 2
echo "Done."
