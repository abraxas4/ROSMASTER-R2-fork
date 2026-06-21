#!/bin/bash
# Show RTAB-Map visualization on the R2's physical monitor while mapping runs elsewhere.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=run_docker_env.sh
source "$SCRIPT_DIR/run_docker_env.sh"

export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" && -f /run/user/1000/gdm/Xauthority ]]; then
  export XAUTHORITY=/run/user/1000/gdm/Xauthority
fi

echo "=== RTAB-Map viz on robot display (${DISPLAY}) ==="
echo "Run this while mapping (이어서/새로) is active in another terminal."

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

exec ros2 launch yahboomcar_nav rtabmap_viz_launch.py