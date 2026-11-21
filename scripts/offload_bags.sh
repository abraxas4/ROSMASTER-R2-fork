#!/usr/bin/env bash
# 완료된 rosbag 분할 파일만 PC 로 보냄. 이 스크립트는 자동으로 시작하지 않음.
# 크기와 sha256 이 양쪽에서 같을 때만 R2 원본을 지움.
# 지금 녹화 중인 파일은 건드리지 않음.
set -euo pipefail

BAG_ROOT="${BAG_ROOT:-/data/bags}"
OFFLOAD_HOST="${OFFLOAD_HOST:-}"
OFFLOAD_USER="${OFFLOAD_USER:-user}"
OFFLOAD_DIR="${OFFLOAD_DIR:-C:/Users/user/r2_bags}"
OFFLOAD_PORT="${OFFLOAD_PORT:-22}"
SSH_OPTS=(-p "${OFFLOAD_PORT}" -o BatchMode=yes -o ConnectTimeout=10)
SCP_OPTS=(-P "${OFFLOAD_PORT}" -o BatchMode=yes -o ConnectTimeout=10)
RECORD_STATE="${BAG_ROOT}/.recording"

usage() {
  cat <<'H'
사용법: offload_bags.sh

환경 변수(또는 아래 기본값)로 대상을 정함.
  BAG_ROOT       기본 /data/bags
  OFFLOAD_HOST   PC 주소 (필수)
  OFFLOAD_USER   PC 계정 (기본 user)
  OFFLOAD_DIR    PC 저장 경로 (기본 C:/Users/user/r2_bags)
  OFFLOAD_PORT   SSH 포트 (기본 22)

완료된 분할(.mcap / .db3)만 보냄. 녹화 중인 파일과 metadata.yaml
(녹화 중일 때)은 건너뜀. 전송 후 양쪽 크기·sha256 이 같으면 R2 파일을 삭제함.
이 스크립트는 녹화나 모터, Nav2 를 시작하지 않음.
H
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -z "${OFFLOAD_HOST}" ]]; then
  echo "오류: OFFLOAD_HOST 를 지정하세요. 예:" >&2
  echo "  OFFLOAD_HOST=192.168.87.10 OFFLOAD_USER=user OFFLOAD_DIR=C:/Users/user/r2_bags $0" >&2
  echo "지금은 실행하지 않고 종료합니다." >&2
  exit 1
fi

REMOTE="${OFFLOAD_USER}@${OFFLOAD_HOST}"
ssh "${SSH_OPTS[@]}" "${REMOTE}" "mkdir -p '${OFFLOAD_DIR}'" </dev/null

# 녹화 중 디렉터리와, 그 안에서 지금 쓰는 데이터 파일.
ACTIVE_DIR=""
ACTIVE_FILE=""
if [[ -f "${RECORD_STATE}" ]]; then
  # shellcheck disable=SC1090
  source "${RECORD_STATE}"
  if [[ -n "${bag_pid:-}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    ACTIVE_DIR="${bag_dir:-}"
  elif [[ -n "${pid:-}" ]] && kill -0 "${pid}" 2>/dev/null; then
    ACTIVE_DIR="${bag_dir:-}"
  fi
fi
# 상태 파일이 없어도 ros2 bag record 가 있으면 보호.
if [[ -z "${ACTIVE_DIR}" ]]; then
  if pgrep -f '[r]os2 bag record' >/dev/null 2>&1; then
    # 가장 최근에 수정된 mapping_* 디렉터리를 활성으로 봄
    ACTIVE_DIR="$(find "${BAG_ROOT}" -mindepth 1 -maxdepth 1 -type d -name 'mapping_*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1{print $2}')"
  fi
fi
if [[ -n "${ACTIVE_DIR}" && -d "${ACTIVE_DIR}" ]]; then
  # 진행 중인 분할: 열려 있거나, 이름이 가장 큰 데이터 파일.
  if command -v lsof >/dev/null 2>&1; then
    ACTIVE_FILE="$(lsof +D "${ACTIVE_DIR}" 2>/dev/null | awk '/\.(mcap|db3)$/{print $NF}' | sort -u | tail -1 || true)"
  fi
  if [[ -z "${ACTIVE_FILE}" ]]; then
    ACTIVE_FILE="$(ls -1 "${ACTIVE_DIR}"/*.[0-9]*.mcap "${ACTIVE_DIR}"/*.[0-9]*.db3 "${ACTIVE_DIR}"/*.mcap "${ACTIVE_DIR}"/*.db3 2>/dev/null | sort -V | tail -1 || true)"
  fi
  echo "녹화 중: ${ACTIVE_DIR}"
  echo "건드리지 않을 파일: ${ACTIVE_FILE:-알 수 없음} 및 metadata.yaml"
fi

is_active() {
  local f="$1"
  [[ -n "${ACTIVE_DIR}" && "${f}" == "${ACTIVE_DIR}/metadata.yaml" ]] && return 0
  [[ -n "${ACTIVE_FILE}" && "${f}" == "${ACTIVE_FILE}" ]] && return 0
  return 1
}

sha256_local() {
  sha256sum "$1" | awk '{print $1}'
}

remote_stat_hash() {
  local dest="$1"
  # POSIX(sha256sum)를 먼저 시도하고, 없으면 Windows certutil.
  ssh "${SSH_OPTS[@]}" "${REMOTE}" "if command -v sha256sum >/dev/null 2>&1; then
      sz=\$(wc -c < '${dest}' | tr -d ' ')
      hs=\$(sha256sum '${dest}' | awk '{print \$1}')
      printf '%s %s\n' \"\$sz\" \"\$hs\"
    elif command -v certutil >/dev/null 2>&1; then
      sz=\$(wc -c < '${dest}' 2>/dev/null || powershell.exe -NoProfile -Command \"(Get-Item -LiteralPath '${dest}').Length\")
      hs=\$(certutil -hashfile '${dest}' SHA256 | awk 'NR==2{print \$1}')
      printf '%s %s\n' \"\$sz\" \"\$hs\"
    else
      echo 'NOHASH'
      exit 2
    fi" </dev/null
}

offload_file() {
  local src="$1"
  local rel="$2"
  local dest="${OFFLOAD_DIR%/}/${rel}"
  local dest_dir
  dest_dir="$(dirname "${dest}")"
  local tmp="${dest}.part"

  if is_active "${src}"; then
    echo "건너뜀(녹화 중): ${src}"
    return 0
  fi

  local lsize lhash
  lsize="$(wc -c < "${src}" | tr -d ' ')"
  lhash="$(sha256_local "${src}")"
  echo "전송: ${src}  (${lsize} bytes, sha256=${lhash})"

  ssh "${SSH_OPTS[@]}" "${REMOTE}" "mkdir -p '${dest_dir}'" </dev/null
  scp "${SCP_OPTS[@]}" "${src}" "${REMOTE}:${tmp}"
  ssh "${SSH_OPTS[@]}" "${REMOTE}" "mv -f '${tmp}' '${dest}'" </dev/null

  local remote_info rsize rhash
  remote_info="$(remote_stat_hash "${dest}")"
  rsize="${remote_info%% *}"
  rhash="${remote_info#* }"
  if [[ "${rsize}" == "${lsize}" && "${rhash}" == "${lhash}" ]]; then
    rm -f "${src}"
    echo "확인됨, R2 원본 삭제: ${src}"
    echo "  원격 ${dest}  size=${rsize} sha256=${rhash}"
    return 0
  fi
  echo "오류: 확인 실패, R2 파일을 유지합니다." >&2
  echo "  local  size=${lsize} sha256=${lhash}" >&2
  echo "  remote size=${rsize} sha256=${rhash}" >&2
  return 1
}

shopt -s nullglob
errors=0
sent=0
skipped=0

# 분할 데이터만. metadata.yaml 은 녹화가 끝난 디렉터리에서만 보냄.
mapfile -t bags < <(find "${BAG_ROOT}" -mindepth 1 -maxdepth 1 -type d -name 'mapping_*' | sort)
for dir in "${bags[@]}"; do
  base="$(basename "${dir}")"
  for f in "${dir}"/*.mcap "${dir}"/*.db3; do
    [[ -f "${f}" ]] || continue
    if is_active "${f}"; then
      echo "건너뜀(녹화 중): ${f}"
      skipped=$((skipped + 1))
      continue
    fi
    rel="${base}/$(basename "${f}")"
    if offload_file "${f}" "${rel}"; then
      sent=$((sent + 1))
    else
      errors=$((errors + 1))
    fi
  done
  if [[ -n "${ACTIVE_DIR}" && "${dir}" == "${ACTIVE_DIR}" ]]; then
    echo "건너뜀(녹화 중): ${dir}/metadata.yaml"
    skipped=$((skipped + 1))
    continue
  fi
  if [[ -f "${dir}/metadata.yaml" ]]; then
    if offload_file "${dir}/metadata.yaml" "${base}/metadata.yaml"; then
      sent=$((sent + 1))
      # 디렉터리가 비었으면 제거
      rmdir "${dir}" 2>/dev/null || true
    else
      errors=$((errors + 1))
    fi
  else
    rmdir "${dir}" 2>/dev/null || true
  fi
done

echo "완료: 전송 ${sent} 개, 건너뜀 ${skipped} 개, 실패 ${errors} 개"
exit $(( errors > 0 ? 1 : 0 ))
