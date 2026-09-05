#!/bin/bash
# Mapping is Cartographer. Keep this name so old shortcuts still work.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/start_cartographer.sh"
