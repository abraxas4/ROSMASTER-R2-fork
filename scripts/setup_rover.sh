#!/bin/bash
# ROSMASTER R2 - 로버(Jetson Orin) 초기 설정 스크립트
# 이 스크립트는 git clone 후 한 번만 실행합니다.

set -e

echo "=== ROSMASTER-R2-fork 초기 설정 (Jetson Orin) ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Repo 위치: $REPO_ROOT"

# 1. 실행 권한 부여
echo "[1/5] 스크립트 실행 권한 설정..."
chmod +x "$SCRIPT_DIR"/*.sh

# 2. split large files 복원
echo "[2/5] split large files 복원..."
bash "$SCRIPT_DIR/restore_large_files.sh"

# 3. 워크스페이스 디렉토리 확인 (이미 git 안에 있음)
WORKSPACE_PATH="$REPO_ROOT/code/yahboomcar_ros2_ws"
echo "[3/5] 워크스페이스 경로: $WORKSPACE_PATH"

PKG_COUNT=$(find "$WORKSPACE_PATH/src" -mindepth 1 -maxdepth 1 ! -name 'README.md' -type d 2>/dev/null | wc -l)
if [ "$PKG_COUNT" -eq 0 ]; then
    echo "WARNING: $WORKSPACE_PATH/src 에 ROS2 패키지가 없습니다."
    echo "다음 명령으로 Google Drive Code를 설치하세요:"
    echo "  bash $SCRIPT_DIR/download_code.sh"
fi

# 3. 호스트에 필요한 데이터 폴더 생성 (기존 마운트 호환)
echo "[4/5] 호스트 데이터 폴더 생성..."
USER_HOME=$(eval echo ~$USER)
mkdir -p "$USER_HOME/temp"
mkdir -p "$USER_HOME/rosboard"
mkdir -p "$USER_HOME/maps"

# 4. 안내
echo "[5/5] 설정 완료!"
echo ""
echo "다음 단계:"
echo "1. ROS2 코드 설치:"
echo "   bash $SCRIPT_DIR/download_code.sh"
echo "   (브라우저로 zip을 받았다면 --from-zip 옵션 사용)"
echo ""
echo "2. Docker 실행:"
echo "   bash $SCRIPT_DIR/run_docker.sh"
echo ""
echo "3. 이후 코드 업데이트는:"
echo "   bash $SCRIPT_DIR/update_code.sh"
echo ""
echo "주의: run_docker.sh 안의 DOCKER_IMAGE 태그와 장치(/dev/...)를 실제 환경에 맞게 수정하세요."
