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
echo ""

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

if ! ros2 topic list 2>/dev/null | grep -qx /rgbd_image; then
  echo "WARN: /rgbd_image not found — start mapping FIRST, then open this view."
  echo "      (매핑 이어서/새로 아이콘을 먼저 실행하세요)"
  sleep 3
fi

VIZ_LAUNCH="$SCRIPT_DIR/launch/rtabmap_viz_managed.launch.py"
exec ros2 launch "$VIZ_LAUNCH"