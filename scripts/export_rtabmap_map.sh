#!/bin/bash
# Export 2D occupancy grid from a running RTAB-Map /map topic for Nav2.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

REG_OUT="$(python3 "$SCRIPT_DIR/map_registry.py" continue)"
eval "$(echo "$REG_OUT" | grep -E '^ACTIVE_MAP_')"

MAP_ID="${ACTIVE_MAP_ID:?no active map}"
DISPLAY_NAME="${ACTIVE_MAP_NAME:-$MAP_ID}"

OUT_DIR="${HOME}/maps/exported/${MAP_ID}"
OUT_BASE="${OUT_DIR}/nav_map"
mkdir -p "$OUT_DIR"

set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash"

wait_for_map() {
  local timeout="${1:-90}"
  local elapsed=0
  while (( elapsed < timeout )); do
    for topic in /map /grid_prob_map; do
      if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
        if timeout 8 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
          echo "Map topic ready: ${topic}"
          return 0
        fi
      fi
    done
    sleep 2
    elapsed=$((elapsed + 2))
  done
  return 1
}

echo "=== Export RTAB-Map grid ==="
echo "Map: ${DISPLAY_NAME} (${MAP_ID})"
echo "Out: ${OUT_BASE}"

if ! wait_for_map "${MAP_EXPORT_WAIT_SEC:-90}"; then
  echo "ERROR: /map not publishing."
  echo "Start localization first (R2 순찰 시작 does this automatically)."
  exit 1
fi

echo "Saving occupancy grid via map_saver_cli..."
ros2 run nav2_map_server map_saver_cli \
  -f "${OUT_BASE}" \
  --ros-args -p save_map_timeout:=120.0

if [[ ! -f "${OUT_BASE}.yaml" || ! -f "${OUT_BASE}.pgm" ]]; then
  echo "ERROR: export failed — expected ${OUT_BASE}.yaml and .pgm"
  exit 1
fi

python3 "$SCRIPT_DIR/map_registry.py" set-export "${MAP_ID}" "${OUT_BASE}.yaml" "${OUT_BASE}.pgm"

echo "Done: ${OUT_BASE}.yaml"