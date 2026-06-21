#!/bin/bash
# Start a NEW RTAB-Map session named by estimated lat/lon (rename later to 집/회사/...).

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/rtabmap_mapping_core.sh" new