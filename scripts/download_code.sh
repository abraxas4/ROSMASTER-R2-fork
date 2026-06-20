#!/bin/bash
# ROSMASTER R2 - Google Drive ROS2 Code 다운로드 및 src/ 설치
#
# 사용법:
#   bash scripts/download_code.sh                     # Google Drive에서 자동 다운로드 시도
#   bash scripts/download_code.sh --from-zip PATH.zip # 브라우저로 받은 zip 직접 지정
#   bash scripts/download_code.sh --from-dir PATH     # 압축 해제된 폴더 직접 지정

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$REPO_ROOT/code/yahboomcar_ros2_ws/src"
TMP_DIR="${TMPDIR:-/tmp}/rosmaster-r2-code-$$"

ROS2_CODE_FOLDER_ID="16rQNu4VPo2gbbTZcIk1qDcX1z5Jqf7yH"
ROS2_CODE_ZIP_ID="1IgohdB66u47zObk3OAVP58SviepjIFX1"
DRIVE_LINK="https://drive.google.com/drive/folders/1liW0vHoQVXqhjQ10amefKF0-oyGHBqj-"
PLATFORM="${PLATFORM:-orin}"

FROM_ZIP=""
FROM_DIR=""

usage() {
    cat <<EOF
Usage:
  bash scripts/download_code.sh
  bash scripts/download_code.sh --from-zip /path/to/ROSMASTER-R2-ROS2-Code.zip
  bash scripts/download_code.sh --from-dir /path/to/extracted/yahboomcar_ros2_ws
  PLATFORM=orin bash scripts/download_code.sh --from-zip /path/to/ROSMASTER-R2-ROS2-Code-002.zip

Google Drive (브라우저):
  $DRIVE_LINK
  -> 5.Code -> ROS2-Code -> ROSMASTER-R2-ROS2-Code.zip (권장, 작은 파일)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --from-zip)
            FROM_ZIP="${2:-}"
            shift 2
            ;;
        --from-dir)
            FROM_DIR="${2:-}"
            shift 2
            ;;
        --platform)
            PLATFORM="${2:-orin}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

cleanup() {
    rm -rf "$TMP_DIR"
}
trap cleanup EXIT

mkdir -p "$SRC_DIR" "$TMP_DIR"

count_packages() {
    find "$SRC_DIR" -mindepth 1 -maxdepth 1 ! -name 'README.md' -type d 2>/dev/null | wc -l
}

install_from_src_tree() {
    local source_src="$1"

    if [[ ! -d "$source_src" ]]; then
        echo "ERROR: source src directory not found: $source_src"
        exit 1
    fi

    local pkg_count
    pkg_count="$(find "$source_src" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)"
    if [[ "$pkg_count" -eq 0 ]]; then
        echo "ERROR: no package directories found in $source_src"
        exit 1
    fi

    echo "Installing packages from: $source_src"
    echo "Target: $SRC_DIR"

    shopt -s dotglob nullglob
    for item in "$source_src"/*; do
        base="$(basename "$item")"
        if [[ "$base" == "README.md" ]]; then
            continue
        fi
        rm -rf "$SRC_DIR/$base"
        cp -a "$item" "$SRC_DIR/$base"
    done
    shopt -u dotglob nullglob

    echo "Installed $(count_packages) package director(ies) into src/"
}

resolve_src_tree() {
    local root="$1"
    local src_child_count

    if [[ -d "$root/src" ]]; then
        src_child_count="$(find "$root/src" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)"
        if [[ -d "$root/src/yahboom_rosmaster_driver" || -d "$root/src/yahboomcar_bringup" || "$src_child_count" -gt 1 ]]; then
            echo "$root/src"
            return 0
        fi
    fi

    if [[ -d "$root/yahboomcar_ros2_ws/yahboomcar_ws/src" ]]; then
        echo "$root/yahboomcar_ros2_ws/yahboomcar_ws/src"
        return 0
    fi

    if [[ -d "$root/yahboomcar_ros2_ws/src" ]]; then
        echo "$root/yahboomcar_ros2_ws/src"
        return 0
    fi

    if [[ -d "$root/ROS2-Code/yahboomcar_ros2_ws/src" ]]; then
        echo "$root/ROS2-Code/yahboomcar_ros2_ws/src"
        return 0
    fi

    local candidate
    candidate="$(find "$root" -type d -path '*/yahboomcar_ros2_ws/yahboomcar_ws/src' 2>/dev/null | head -1)"
    if [[ -n "$candidate" ]]; then
        echo "$candidate"
        return 0
    fi

    candidate="$(find "$root" -type d -path '*/yahboomcar_ros2_ws/src' 2>/dev/null | head -1)"
    if [[ -n "$candidate" ]]; then
        echo "$candidate"
        return 0
    fi

    if [[ -d "$root/src" ]]; then
        echo "$root/src"
        return 0
    fi

    return 1
}

extract_nested_workspace_zip() {
    local root="$1"
    local platform_dir=""
    local nested_zip=""

    case "$PLATFORM" in
        orin|orin-super|jetson-orin)
            platform_dir="$(find "$root" -maxdepth 1 -type d -iname 'For jetson orin super' 2>/dev/null | head -1)"
            ;;
        nano|pi5|nano-pi5)
            platform_dir="$(find "$root" -maxdepth 1 -type d -iname 'For jetson nano-pi5' 2>/dev/null | head -1)"
            ;;
    esac

    if [[ -n "$platform_dir" && -f "$platform_dir/yahboomcar_ros2_ws.zip" ]]; then
        nested_zip="$platform_dir/yahboomcar_ros2_ws.zip"
    else
        nested_zip="$(find "$root" -type f -name 'yahboomcar_ros2_ws.zip' 2>/dev/null | head -1)"
    fi

    if [[ -n "$nested_zip" ]]; then
        echo "Extracting nested workspace: $nested_zip"
        unzip -q "$nested_zip" -d "$root"
    fi
}

install_from_zip() {
    local zip_path="$1"

    if [[ ! -f "$zip_path" ]]; then
        echo "ERROR: zip file not found: $zip_path"
        exit 1
    fi

    echo "Extracting: $zip_path"
    unzip -q "$zip_path" -d "$TMP_DIR"
    extract_nested_workspace_zip "$TMP_DIR"

    local source_src
    if ! source_src="$(resolve_src_tree "$TMP_DIR")"; then
        echo "ERROR: could not find yahboomcar_ros2_ws/src inside archive"
        echo "Archive top-level contents:"
        ls -la "$TMP_DIR"
        exit 1
    fi

    install_from_src_tree "$source_src"
}

install_from_dir() {
    local dir_path="$1"

    if [[ ! -d "$dir_path" ]]; then
        echo "ERROR: directory not found: $dir_path"
        exit 1
    fi

    local source_src
    if ! source_src="$(resolve_src_tree "$dir_path")"; then
        if [[ -d "$dir_path" ]] && [[ $(find "$dir_path" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l) -gt 0 ]]; then
            source_src="$dir_path"
        else
            echo "ERROR: could not find src packages in: $dir_path"
            exit 1
        fi
    fi

    install_from_src_tree "$source_src"
}

download_with_gdown() {
    if ! command -v gdown >/dev/null 2>&1; then
        echo "Installing gdown..."
        python3 -m pip install --user gdown
        export PATH="$HOME/.local/bin:$PATH"
    fi

    local zip_path="$TMP_DIR/ROSMASTER-R2-ROS2-Code.zip"

    echo "Trying Google Drive download (ROSMASTER-R2-ROS2-Code.zip)..."
    if gdown "https://drive.google.com/uc?id=$ROS2_CODE_ZIP_ID" -O "$zip_path"; then
        install_from_zip "$zip_path"
        return 0
    fi

    echo "Single-file download failed. Trying ROS2-Code folder..."
    if gdown --folder "https://drive.google.com/drive/folders/$ROS2_CODE_FOLDER_ID" -O "$TMP_DIR" 2>/dev/null; then
        local folder_zip
        folder_zip="$(find "$TMP_DIR" -maxdepth 2 -name 'ROSMASTER-R2-ROS2-Code.zip' | head -1)"
        if [[ -n "$folder_zip" ]]; then
            install_from_zip "$folder_zip"
            return 0
        fi
    fi

    return 1
}

print_manual_steps() {
    cat <<EOF

Google Drive 자동 다운로드에 실패했습니다.
(원인: 다운로드 한도 초과 또는 네트워크 제한일 수 있습니다)

브라우저로 직접 받은 뒤 아래 명령을 실행하세요:

  1) $DRIVE_LINK
  2) 5.Code -> ROS2-Code -> ROSMASTER-R2-ROS2-Code.zip 다운로드
  3) bash scripts/download_code.sh --from-zip ~/Downloads/ROSMASTER-R2-ROS2-Code.zip

이미 압축을 풀었다면:

  bash scripts/download_code.sh --from-dir /path/to/yahboomcar_ros2_ws

EOF
}

main() {
    echo "=== ROSMASTER R2 Code Install ==="
    echo "Workspace src: $SRC_DIR"

    if [[ -n "$FROM_ZIP" ]]; then
        install_from_zip "$FROM_ZIP"
    elif [[ -n "$FROM_DIR" ]]; then
        install_from_dir "$FROM_DIR"
    else
        if ! download_with_gdown; then
            print_manual_steps
            exit 1
        fi
    fi

    echo ""
    echo "Done. Next steps:"
    echo "  git add code/yahboomcar_ros2_ws/src/"
    echo "  git commit -m \"Add ROS2 packages from Google Drive 5.Code\""
    echo "  git push origin main"
}

main