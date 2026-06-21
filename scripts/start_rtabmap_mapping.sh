#!/bin/bash
# Deprecated wrapper — use start_rtabmap_continue.sh or start_rtabmap_new.sh.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "NOTE: 이 스크립트는 구버전입니다."
echo "      바탕화면 'R2 매핑 이어서 (카메라+라이다)' 아이콘을 사용하세요."
echo ""

exec bash "$SCRIPT_DIR/start_rtabmap_continue.sh"