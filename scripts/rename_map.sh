#!/bin/bash
# Rename the active map to a friendly label (집, 회사, 광교역회사, ...).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: bash $0 '표시이름'"
  echo ""
  echo "Examples:"
  echo "  bash $0 '집'"
  echo "  bash $0 '광교역회사'"
  echo ""
  python3 "$SCRIPT_DIR/map_registry.py" list
  exit 1
fi

python3 "$SCRIPT_DIR/map_registry.py" rename "$*"