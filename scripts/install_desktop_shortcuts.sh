#!/bin/bash
# Install R2 desktop shortcuts and remove obsolete icons.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESKTOP_SRC="$SCRIPT_DIR/desktop"
DESKTOP_DST="${HOME}/Desktop"

OBSOLETE=(
  R2_RTABMap_Mapping.desktop
  R2_RTABMap_View.desktop
  R2_Mapping.desktop
  R2_RViz.desktop
  R2_Lidar_Mapping.desktop
  R2_Map_Continue.desktop
)

if [[ ! -d "$DESKTOP_SRC" ]]; then
  echo "ERROR: missing $DESKTOP_SRC"
  exit 1
fi

mkdir -p "$DESKTOP_DST"

for name in "${OBSOLETE[@]}"; do
  if [[ -f "$DESKTOP_DST/$name" ]]; then
    rm -f "$DESKTOP_DST/$name"
    echo "Removed obsolete: $DESKTOP_DST/$name"
  fi
  if [[ -f "$DESKTOP_SRC/$name" ]]; then
    rm -f "$DESKTOP_SRC/$name"
    echo "Removed from repo: $DESKTOP_SRC/$name"
  fi
done

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
echo "Desktop shortcuts:"
echo "  매핑 시작 / 매핑 중지 / 순찰 시작 / 순찰 중지"
