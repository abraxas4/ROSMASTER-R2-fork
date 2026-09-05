#!/bin/bash
# Save Cartographer state if running, convert pbstream to Nav2 yaml/pgm, register.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERT_BIN="${CONVERT_BIN:-/opt/ros/humble/lib/cartographer_ros/cartographer_pbstream_to_ros_map}"
OUTDIR="${HOME}/maps/cartographer"
RESOLUTION="${RESOLUTION:-0.05}"

mkdir -p "$OUTDIR"

PBSTREAM="${1:-}"

if pgrep -x cartographer_node >/dev/null 2>&1; then
  set +u
  # shellcheck source=native_ros_setup.bash
  source "$SCRIPT_DIR/native_ros_setup.bash"
  set -u
  MAP_ID="$(date +%Y%m%d_%H%M%S)"
  PBSTREAM="${OUTDIR}/${MAP_ID}.pbstream"
  echo "Writing Cartographer state -> ${PBSTREAM}"
  if ! timeout 25 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '${PBSTREAM}', include_unfinished_submaps: true}"; then
    echo "ERROR: /write_state failed"
    exit 1
  fi
  if [[ ! -f "$PBSTREAM" ]]; then
    echo "ERROR: pbstream was not written: ${PBSTREAM}"
    exit 1
  fi
fi

if [[ -z "$PBSTREAM" ]]; then
  PBSTREAM="$(ls -1t "${OUTDIR}"/*.pbstream 2>/dev/null | head -1 || true)"
fi

if [[ -z "$PBSTREAM" || ! -f "$PBSTREAM" ]]; then
  echo "ERROR: no Cartographer pbstream in ${OUTDIR}"
  exit 1
fi

STEM="${PBSTREAM%.pbstream}"
YAML_PATH="${STEM}.yaml"
PGM_PATH="${STEM}.pgm"

if [[ ! -f "$YAML_PATH" || ! -f "$PGM_PATH" ]]; then
  if [[ ! -x "$CONVERT_BIN" ]]; then
    echo "ERROR: missing ${CONVERT_BIN}"
    exit 1
  fi
  echo "Converting ${PBSTREAM} -> yaml/pgm"
  "$CONVERT_BIN" \
    -pbstream_filename "$PBSTREAM" \
    -map_filestem "$STEM" \
    -resolution "$RESOLUTION"
fi

if [[ ! -f "$YAML_PATH" ]]; then
  echo "ERROR: conversion produced no yaml: ${YAML_PATH}"
  exit 1
fi

python3 "$SCRIPT_DIR/map_registry.py" register \
  --id "$(basename "$STEM")" \
  --pbstream "$PBSTREAM" \
  --yaml "$YAML_PATH" \
  --pgm "$PGM_PATH"

echo "Saved Nav2 map: ${YAML_PATH}"
