#!/bin/bash
# Install R2 mapping desktop shortcuts to ~/Desktop on the rover.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESKTOP_SRC="$SCRIPT_DIR/desktop"
DESKTOP_DST="${HOME}/Desktop"

if [[ ! -d "$DESKTOP_SRC" ]]; then
  echo "ERROR: missing $DESKTOP_SRC"
  exit 1
fi

mkdir -p "$DESKTOP_DST"

for entry in "$DESKTOP_SRC"/*.desktop; do
  [[ -f "$entry" ]] || continue
  name="$(basename "$entry")"
  cp -f "$entry" "$DESKTOP_DST/$name"
  chmod +x "$DESKTOP_DST/$name"
  if command -v gio >/dev/null 2>&1; then
    gio set "$DESKTOP_DST/$name" metadata::trusted true 2>/dev/null || true
  fi
  echo "Installed: $DESKTOP_DST/$name"
done

echo ""
echo "Desktop shortcuts ready. Usage:"
echo "  1) 'R2 매핑 이어서하기' — append to active map"
echo "  2) 'R2 매핑 새로하기' — new map (auto lat/lon name)"
echo "  3) 'R2 맵 실시간 보기' — live map on this screen"
echo "  4) 'R2 매핑 중지' — stop nodes safely"
echo "  5) 'R2 순찰 시작' — geofence patrol (after mapping)"
echo "  6) 'R2 순찰 중지'"
echo "  Rename: bash ~/ROSMASTER-R2-fork/scripts/rename_map.sh '집'"