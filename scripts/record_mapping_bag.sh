#!/usr/bin/env bash
# 맵 작성·모방학습용 rosbag 녹화. Ctrl-C 로 종료.
# /cmd_vel 을 발행하지 않고, Nav2 목표나 탐색도 시작하지 않음.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_ROOT="${BAG_ROOT:-/data/bags}"
MIN_FREE_GB="${MIN_FREE_GB:-5}"
MAX_BAG_DURATION="${MAX_BAG_DURATION:-60}"
REPO="${REPO:-$HOME/ROSMASTER-R2-fork}"
FORK_WS="${FORK_WS:-$REPO/code/yahboomcar_ros2_ws}"
LIBRARY_WS="${LIBRARY_WS:-$HOME/yahboomcar_ros2_ws/software/library_ws}"
RECORD_STATE="${BAG_ROOT}/.recording"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${LIBRARY_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${LIBRARY_WS}/install/setup.bash"
fi
if [[ -f "${FORK_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${FORK_WS}/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

mkdir -p "${BAG_ROOT}"

free_bytes="$(df -B1 --output=avail "${BAG_ROOT}" | tail -1 | tr -d ' ')"
free_gb="$(awk -v b="${free_bytes}" 'BEGIN { printf "%.1f", b/1024/1024/1024 }')"
if awk -v b="${free_bytes}" -v min="${MIN_FREE_GB}" 'BEGIN { exit (b >= min*1024*1024*1024) ? 0 : 1 }'; then
  echo "디스크 여유: ${free_gb} GB (${BAG_ROOT})"
else
  echo "오류: ${BAG_ROOT} 여유 공간이 ${free_gb} GB 뿐입니다. ${MIN_FREE_GB} GB 이상일 때만 녹화합니다." >&2
  exit 1
fi

# 카메라가 꺼져 있으면 라이다·odom 위주. 카메라(640x480, 30 Hz 원본)까지 넣으면
# 수백 MB/min ~ 수 GB/min 까지 늘어날 수 있음. 압축 토픽이 있으면 그걸 씀.
echo "용량 추정: 카메라 영상이 없으면 분당 수십 MB 수준(실측은 녹화 후 ros2 bag info)."
echo "           카메라 원본 RGB+깊이가 붙으면 분당 수 GB 가까이 늘 수 있어, 압축 토픽을 우선합니다."
awk -v free="${free_bytes}" 'BEGIN {
  # 예시값: 카메라 없음 40 MB/min, 압축 영상 200 MB/min, 원본 영상 3000 MB/min
  for (mb in m) {}
  split("40 200 3000", rates, " ")
  names[1]="카메라 없음(예시값)"; names[2]="압축 영상(예시값)"; names[3]="원본 영상(예시값)"
  for (i=1;i<=3;i++) {
    min = free / (rates[i]*1024*1024)
    printf "  - %s %d MB/min 이면 약 %.0f 분 녹화 가능\n", names[i], rates[i], min
  }
}'

topic_exists() {
  local t="$1"
  printf '%s\n' "${TOPIC_LIST}" | grep -Fxq -- "${t}"
}

TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
if [[ -z "${TOPIC_LIST}" ]]; then
  echo "오류: ros2 topic list 가 비었습니다. ROS_DOMAIN_ID=${ROS_DOMAIN_ID} 와 실행 중인 스택을 확인하세요." >&2
  exit 1
fi

REQUIRED=(
  /scan
  /tf
  /tf_static
  /odom
  /ackermann_steering_controller/odometry
  /imu/data_raw
  /cmd_vel
  /battery_voltage
  /map
)

missing=()
for t in "${REQUIRED[@]}"; do
  if ! topic_exists "${t}"; then
    missing+=("${t}")
  fi
done
if (( ${#missing[@]} > 0 )); then
  echo "경고: 지금 그래프에 없는 토픽이 있습니다. 나중에 발행되면 같이 기록합니다: ${missing[*]}"
fi

# 이미지: 압축 토픽이 있으면 그걸 쓰고, 없으면 image_raw.
pick_image() {
  local compressed="$1"
  local raw="$2"
  if topic_exists "${compressed}"; then
    printf '%s\n' "${compressed}"
  elif topic_exists "${raw}"; then
    printf '%s\n' "${raw}"
  else
    # 카메라가 나중에 켜질 수 있음. 원본 이름을 예약하고 경고.
    printf '%s\n' "${raw}"
    return 1
  fi
}

CAMERA_TOPICS=()
CAM_WARN=0
if color_img="$(pick_image /camera/color/image_raw/compressed /camera/color/image_raw)"; then
  CAMERA_TOPICS+=("${color_img}")
else
  CAMERA_TOPICS+=("${color_img}")
  CAM_WARN=1
fi
# depth compressedImage or compressedDepth
depth_img=""
if topic_exists /camera/depth/image_raw/compressedDepth; then
  depth_img=/camera/depth/image_raw/compressedDepth
elif topic_exists /camera/depth/image_raw/compressed; then
  depth_img=/camera/depth/image_raw/compressed
elif topic_exists /camera/depth/image_raw; then
  depth_img=/camera/depth/image_raw
else
  depth_img=/camera/depth/image_raw
  CAM_WARN=1
fi
CAMERA_TOPICS+=("${depth_img}")

for info in /camera/color/camera_info /camera/depth/camera_info; do
  CAMERA_TOPICS+=("${info}")
  if ! topic_exists "${info}"; then
    CAM_WARN=1
  fi
done
if topic_exists /camera/depth/points; then
  CAMERA_TOPICS+=(/camera/depth/points)
fi

if (( CAM_WARN == 1 )); then
  echo "경고: Orbbec(astra) 카메라 영상·camera_info 토픽이 지금 없습니다."
  echo "      장치 이름은 /camera/color/image_raw, /camera/depth/image_raw, /camera/*/camera_info 입니다."
  echo "      카메라가 꺼져 있으면 이 토픽은 빈 채로 두고, 켜지면 이어서 기록합니다."
  echo "      카메라는 이 스크립트가 시작하지 않습니다."
fi

TOPICS=("${REQUIRED[@]}" "${CAMERA_TOPICS[@]}")
# unique, keep order
UNIQ=()
for t in "${TOPICS[@]}"; do
  skip=0
  for u in "${UNIQ[@]+"${UNIQ[@]}"}"; do
    [[ "${u}" == "${t}" ]] && { skip=1; break; }
  done
  (( skip == 0 )) && UNIQ+=("${t}")
done
TOPICS=("${UNIQ[@]}")

STORAGE="sqlite3"
if ros2 bag record -h 2>&1 | grep -q -- '--storage' && ros2 bag record -h 2>&1 | grep -E -q '\{.*mcap.*\}'; then
  STORAGE="mcap"
fi

# 파일 단위 zstd 는 1분 분할이 닫힐 때 한 번씩만 압축해서 부담이 작음.
COMP_ARGS=()
if ros2 bag record -h 2>&1 | grep -q -- '--compression-mode'; then
  COMP_ARGS=(--compression-mode file --compression-format zstd)
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${BAG_ROOT}/mapping_${STAMP}"

echo "저장 형식: ${STORAGE}  압축: ${COMP_ARGS[*]:-없음}  분할: ${MAX_BAG_DURATION}s"
echo "출력: ${BAG_DIR}"
echo "토픽:"
printf '  %s\n' "${TOPICS[@]}"
echo "Ctrl-C 로 중지합니다."

cleanup() {
  local ec=$?
  rm -f "${RECORD_STATE}"
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  exit "${ec}"
}
# Background start (foo.sh &) inherits SIG_IGN for INT; restore so TERM/INT work.
trap - INT
trap cleanup INT TERM EXIT

{
  echo "pid=$$"
  echo "bag_dir=${BAG_DIR}"
  echo "started=$(date -Iseconds)"
} > "${RECORD_STATE}"

QOS_FILE="${QOS_FILE:-${SCRIPT_DIR}/params/record_mapping_bag_qos.yaml}"
QOS_ARGS=()
if [[ -f "${QOS_FILE}" ]]; then
  QOS_ARGS=(--qos-profile-overrides-path "${QOS_FILE}")
  echo "QoS 오버라이드: ${QOS_FILE}"
fi

# --include-unpublished-topics: 카메라나 /cmd_vel 이 나중에 떠도 기록
ros2 bag record \
  -s "${STORAGE}" \
  "${COMP_ARGS[@]}" \
  "${QOS_ARGS[@]}" \
  --max-bag-duration "${MAX_BAG_DURATION}" \
  --include-unpublished-topics \
  -o "${BAG_DIR}" \
  "${TOPICS[@]}" &
BAG_PID=$!
# record state should track the bag process too
{
  echo "pid=$$"
  echo "bag_pid=${BAG_PID}"
  echo "bag_dir=${BAG_DIR}"
  echo "started=$(date -Iseconds)"
} > "${RECORD_STATE}"

wait "${BAG_PID}"
BAG_PID=""
rm -f "${RECORD_STATE}"
trap - INT TERM EXIT
echo "녹화 종료: ${BAG_DIR}"
if [[ -d "${BAG_DIR}" ]]; then
  du -sh "${BAG_DIR}" || true
  ros2 bag info "${BAG_DIR}" || true
fi
