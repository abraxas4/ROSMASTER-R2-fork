# scripts/ - Rover에서 편하게 사용하기 위한 도우미 스크립트

## 주요 파일

| 파일 | 용도 |
|------|------|
| `download_code.sh` | Google Drive `5.Code` 다운로드 및 `src/` 설치 |
| `run_docker.sh` | Docker 컨테이너 실행 스크립트 (워크스페이스 마운트 포함) |
| `setup_rover.sh` | 로버(Jetson Orin)에서 처음 한 번 실행할 셋업 스크립트 |
| `update_code.sh` | git pull + 필요한 동기화 (로버에서 주로 사용) |

## 사용 순서 (로버에서)

### 1. 처음 설정 (한 번만)

```bash
cd /home/jetson
git clone https://github.com/abraxas4/ROSMASTER-R2-fork.git
cd ROSMASTER-R2-fork
chmod +x scripts/*.sh
bash scripts/setup_rover.sh
```

### 2. 코드 업데이트할 때 (이게 핵심!)

```bash
cd /home/jetson/ROSMASTER-R2-fork
bash scripts/update_code.sh
```

이 스크립트는:
- `git pull` 실행
- 필요시 workspace 권한 정리
- (선택) 컨테이너 재시작 안내

### 3. Docker 실행

```bash
bash scripts/run_docker.sh
```

컨테이너 안에서:

```bash
cd /root/yahboomcar_ros2_ws
source install/setup.bash
# 또는 colcon build 후 사용
```

## PC에서 작업하는 경우

1. 이 repo를 clone
2. ROS2 코드 최초 설치:
   ```bash
   bash scripts/download_code.sh
   # 또는 브라우저로 받은 zip:
   bash scripts/download_code.sh --from-zip ~/Downloads/ROSMASTER-R2-ROS2-Code.zip
   ```
3. `code/yahboomcar_ros2_ws/src/` 아래 패키지 수정
4. git add/commit/push
5. 로버에서 위 `update_code.sh` 실행

## 주의
- `run_docker.sh` 는 기존 로버 스크립트를 기반으로 수정한 예시입니다.
- USB 장치 경로(/dev/...) 는 실제 연결된 장치에 따라 수정하세요.
- Orin 모델에 따라 user 이름이 `jetson` 또는 `orin` 일 수 있습니다. 필요시 경로를 수정하세요.

## 맵 녹화와 PC 반출

맵을 만들며 돌아다닐 때 카메라·라이다·명령 속도를 같이 남기는 스크립트임. 모터를 움직이지도, Nav2 목표를 보내지도 않음.

### `record_mapping_bag.sh`

```bash
bash scripts/record_mapping_bag.sh
# 중지: Ctrl-C
```

- 기본 저장 위치: `/data/bags/mapping_YYYYMMDD_HHMMSS` (`BAG_ROOT` 로 바꿀 수 있음)
- 여유 공간이 5 GB 미만이면 녹화를 거부함
- 가능하면 mcap, 아니면 sqlite3. 1분마다 파일을 나누고(`--max-bag-duration 60`), 닫힌 파일만 zstd 로 압축함
- 기록 토픽: `/scan`, `/tf`, `/tf_static`, `/odom`, `/ackermann_steering_controller/odometry`, `/imu/data_raw`, `/cmd_vel`, `/battery_voltage`, `/map`, 카메라 RGB·깊이·`camera_info`
- 카메라 토픽은 실행 시 `ros2 topic list` 로 찾고, 압축 토픽이 있으면 그걸 우선함. 지금 카메라가 꺼져 있으면 경고만 하고 이름만 예약함
- `/scan` 과 카메라 토픽은 BEST_EFFORT 이라 `scripts/params/record_mapping_bag_qos.yaml` 로 맞춰 둠. 기본값(RELIABLE)이면 라이다가 0건이 됨
- 백그라운드로 띄웠을 때는 Ctrl-C 대신 `kill -TERM <pid>` (상태 파일 `/data/bags/.recording`)
- 녹화 중 상태 파일은 `/data/bags/.recording`

### `offload_bags.sh`

자동으로 시작하지 않음. 필요할 때 직접 실행함.

```bash
OFFLOAD_HOST=<PC주소> OFFLOAD_USER=user OFFLOAD_DIR=C:/Users/user/r2_bags \
  bash scripts/offload_bags.sh
```

- 완료된 분할 파일만 보냄. 지금 녹화 중인 파일과, 녹화 중인 디렉터리의 `metadata.yaml` 은 건드리지 않음
- 전송 뒤 양쪽에서 크기와 sha256 을 비교하고, 둘 다 같을 때만 R2 원본을 지움
