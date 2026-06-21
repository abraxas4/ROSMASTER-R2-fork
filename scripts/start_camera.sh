#!/bin/bash
# Start Orbbec Astra Plus / FHD-1080p camera via astra_camera (native Humble).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

echo "=== Orbbec camera (astra_camera astro_pro_plus) ==="

if ! ls /dev/AstraPlus /dev/AstraPlus_rgb /dev/astro_pro_plus 2>/dev/null | head -1 >/dev/null; then
  echo "ERROR: Orbbec camera devices not found."
  echo "Check: lsusb | grep -i orbbec"
  exit 1
fi

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

exec ros2 launch astra_camera astro_pro_plus.launch.xml