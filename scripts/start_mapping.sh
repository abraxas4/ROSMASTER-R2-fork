#!/bin/bash
# Start 4ROS lidar SLAM mapping on native ROS2 Humble.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

echo "=== R2 Mapping (4ROS) via native Humble ==="
echo "library_ws: ${LIBRARY_WS_HOST}"
echo "workspace:  ${NATIVE_WS_HOST}"
echo ""
echo "Note: library_ws (lidar + slam_gmapping) runs on host, not Foxy Docker."

if pgrep -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' >/dev/null 2>&1; then
  echo "Stopping rosmaster_main.py (releases /dev/myserial)..."
  pkill -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' || true
  sleep 1
fi

# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

ros2 launch yahboomcar_nav map_gmapping_4ros_launch.py