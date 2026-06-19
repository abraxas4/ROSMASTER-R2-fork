#!/bin/bash
# ROSMASTER R2 - 로버(Jetson Orin) 초기 설정 스크립트
# 이 스크립트는 git clone 후 한 번만 실행합니다.

set -e

echo "=== ROSMASTER-R2-fork 초기 설정 (Jetson Orin) ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Repo 위치: $REPO_ROOT"

# 1. 실행 권한 부여
echo "[1/4] 스크립트 실행 권한 설정..."
chmod +x "$SCRIPT_DIR"/*.sh

# 2. 워크스페이스 디렉토리 확인 (이미 git 안에 있음)
WORKSPACE_PATH="$REPO_ROOT/code/yahboomcar_ros2_ws"
echo "[2/4] 워크스페이스 경로: $WORKSPACE_PATH"

if [ ! -d "$WORKSPACE_PATH/src" ]; then
    echo "WARNING: $WORKSPACE_PATH/src 가 비어있습니다."
    echo "Google Drive의 Code를 복사해서 src/ 안에 넣어주세요."
fi

# 3. 호스트에 필요한 데이터 폴더 생성 (기존 마운트 호환)
echo "[3/4] 호스트 데이터 폴더 생성..."
mkdir -p /home/jetson/temp
mkdir -p /home/jetson/rosboard
mkdir -p /home/jetson/maps

# 4. 안내
echo "[4/4] 설정 완료!"
echo ""
echo "다음 단계:"
echo "1. Google Drive '5.Code' 를 다운로드해서"
echo "   $WORKSPACE_PATH/src/ 안에 복사하세요."
echo ""
echo "2. Docker 실행:"
echo "   bash $SCRIPT_DIR/run_docker.sh"
echo ""
echo "3. 이후 코드 업데이트는:"
echo "   bash $SCRIPT_DIR/update_code.sh"
echo ""
echo "주의: run_docker.sh 안의 DOCKER_IMAGE 태그와 장치(/dev/...)를 실제 환경에 맞게 수정하세요."
