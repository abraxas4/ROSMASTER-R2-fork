# scripts/ - Rover에서 편하게 사용하기 위한 도우미 스크립트

## 주요 파일

| 파일 | 용도 |
|------|------|
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
2. `code/yahboomcar_ros2_ws/src/` 아래 패키지 수정
3. git add/commit/push
4. 로버에서 위 `update_code.sh` 실행

## 주의
- `run_docker.sh` 는 기존 로버 스크립트를 기반으로 수정한 예시입니다.
- USB 장치 경로(/dev/...) 는 실제 연결된 장치에 따라 수정하세요.
- Orin 모델에 따라 user 이름이 `jetson` 또는 `orin` 일 수 있습니다. 필요시 경로를 수정하세요.
