#!/bin/bash
# ROSMASTER R2 - 코드 업데이트 스크립트
# 로버에서 "git pull만 하고 싶을 때" 사용하는 메인 스크립트

set -e

echo "=== ROSMASTER R2 코드 업데이트 (git pull + 동기화) ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

echo "[1/3] git pull 실행..."
git pull --rebase

echo "[2/3] 워크스페이스 권한 정리 (필요시)..."
WORKSPACE="$REPO_ROOT/code/yahboomcar_ros2_ws"

# build/install/log 이 있으면 소유자 정리 (Docker 안에서 빌드한 경우 흔함)
if [ -d "$WORKSPACE/build" ] || [ -d "$WORKSPACE/install" ]; then
    echo "  build/install 폴더 권한 정리 중..."
    sudo chown -R $USER:$USER "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log" 2>/dev/null || true
fi

echo "[3/3] 완료!"
echo ""
echo "다음 명령으로 Docker를 다시 시작하세요 (필요한 경우):"
echo "  bash $SCRIPT_DIR/run_docker.sh"
echo ""
echo "컨테이너 안에서 빌드하려면:"
echo "  cd /root/yahboomcar_ros2_ws"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
echo "업데이트가 완료되었습니다. (git pull 기반)"
