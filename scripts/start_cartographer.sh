#!/bin/bash
# 2D Cartographer mapping (lidar only).

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LIBRARY_WS_HOST="${LIBRARY_WS_HOST:-$HOME/yahboomcar_ros2_ws/software/library_ws}"
export NATIVE_WS_HOST="${NATIVE_WS_HOST:-$HOME/yahboomcar_ros2_ws/yahboomcar_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
export ROBOT_TYPE="${ROBOT_TYPE:-r2}"
export RPLIDAR_TYPE="${RPLIDAR_TYPE:-4ROS}"

bash "$SCRIPT_DIR/stop_robot_stack.sh"

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"
set -u

echo "=== R2 Cartographer mapping (lidar only) ==="
echo "Joystick: button 2 / MODE, then left stick."
echo "Stop: bash $SCRIPT_DIR/stop_robot_stack.sh  (saves the map first)"
echo ""

on_stop() {
  trap - EXIT INT TERM
  bash "$SCRIPT_DIR/stop_robot_stack.sh"
}
trap on_stop EXIT INT TERM

touch /tmp/r2_serial_for_driver
sleep 0.4

ros2 launch "$SCRIPT_DIR/launch/cartographer_managed.launch.py"
