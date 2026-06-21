#!/bin/bash
# Continue RTAB-Map on the active map (does NOT delete existing database).

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/rtabmap_mapping_core.sh" continue