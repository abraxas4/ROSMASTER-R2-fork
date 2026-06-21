#!/bin/bash
# Export 2D occupancy grid from the active RTAB-Map database for Nav2.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

REG_OUT="$(python3 "$SCRIPT_DIR/map_registry.py" continue)"
eval "$(echo "$REG_OUT" | grep -E '^ACTIVE_MAP_')"

MAP_ID="${ACTIVE_MAP_ID:?no active map}"
DB_PATH="${ACTIVE_MAP_DB:?no active db}"
DISPLAY_NAME="${ACTIVE_MAP_NAME:-$MAP_ID}"

OUT_DIR="${HOME}/maps/exported/${MAP_ID}"
OUT_BASE="${OUT_DIR}/nav_map"
mkdir -p "$OUT_DIR"

if [[ ! -f "$DB_PATH" ]]; then
  echo "ERROR: database not found: $DB_PATH"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${HOME}/yahboomcar_ros2_ws/software/library_ws/install/setup.bash" 2>/dev/null || true

echo "=== Export RTAB-Map grid ==="
echo "Map: ${DISPLAY_NAME} (${MAP_ID})"
echo "DB:  ${DB_PATH}"
echo "Out: ${OUT_BASE}"

rtabmap-export --grid "$DB_PATH" "$OUT_BASE"

python3 "$SCRIPT_DIR/map_registry.py" set-export "${MAP_ID}" "${OUT_BASE}.yaml" "${OUT_BASE}.pgm"

echo "Done: ${OUT_BASE}.yaml"