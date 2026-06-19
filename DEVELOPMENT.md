# Git으로 ROSMASTER R2 코드 관리하기 (Jetson Orin)

이 문서는 **PC에서 수정하고, 로버에서는 git pull만으로 업데이트** 받는 방법을 설명합니다.

## 목표

- Upstream(Yahboom)은 절대 수정하지 않음
- 당신의 fork (abraxas4/ROSMASTER-R2-fork) 에만 코드 commit & push
- 로버(Jetson Orin)에서는 `git pull` + 한 번의 스크립트 실행으로 최신 코드 적용

## 전체 구조 (이 repo 안)

```
ROSMASTER-R2-fork/
├── (기존 PDF 튜토리얼들)
├── code/
│   └── yahboomcar_ros2_ws/          ← 실제 ROS2 코드가 들어갈 곳
│       ├── src/                     ← Google Drive Code의 src 내용 복사
│       └── .gitignore
├── scripts/
│   ├── run_docker.sh                ← Docker 실행 (워크스페이스 마운트 포함)
│   ├── setup_rover.sh
│   └── update_code.sh               ← 로버에서 주로 사용하는 업데이트 스크립트
├── DEVELOPMENT.md
└── README.md
```

## 1. PC (이 컴퓨터) 에서 하는 일

```bash
cd ~/github/ROSMASTER-R2-fork

# 1. 최신 upstream 가져오기 (선택)
git fetch upstream
git merge upstream/main

# 2. Google Drive Code 최초 복사 (한 번)
#    code/yahboomcar_ros2_ws/src/ 안에 패키지들 넣기

# 3. 수정 후
git add .
git commit -m "기능 추가: XXX"
git push origin main
```

## 2. 로버(Jetson Orin) 에서 하는 일

### 최초 설정 (한 번)

```bash
cd /home/jetson
git clone https://github.com/abraxas4/ROSMASTER-R2-fork.git
cd ROSMASTER-R2-fork

chmod +x scripts/*.sh
bash scripts/setup_rover.sh
```

Google Drive Code를 아직 안 넣었다면:
- PC에서 `code/yahboomcar_ros2_ws/src/` 에 넣고 push
- 또는 로버에서 직접 복사

### 일상 업데이트 (git pull만!)

```bash
cd /home/jetson/ROSMASTER-R2-fork
bash scripts/update_code.sh
```

`update_code.sh` 가:
- `git pull`
- build 폴더 권한 정리
- 다음에 Docker 들어가서 `colcon build` 하라고 안내

### Docker 실행 (수정된 스크립트)

```bash
bash scripts/run_docker.sh
```

이 스크립트는 `code/yahboomcar_ros2_ws` 를 컨테이너의 `/root/yahboomcar_ros2_ws` 로 마운트합니다.

컨테이너 안에서:

```bash
cd /root/yahboomcar_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Docker 스크립트 수정 포인트 (중요)

`scripts/run_docker.sh` 에서 다음을 실제 환경에 맞게 고치세요:

1. `DOCKER_IMAGE` 태그 (현재 사용 중인 이미지 확인: `docker images`)
2. `--device=/dev/xxx` 라인 (실제 lidar, camera, serial 포트)
3. user 경로 (`/home/jetson` → `orin` 등인 경우)

한 번만 맞추면 이후 `git pull` 로 스크립트도 업데이트됩니다.

## 4. Google Drive Code를 git에 넣는 방법

1. Google Drive → `5.Code` 다운로드
2. `yahboomcar_ros2_ws` 압축 풀기
3. `src/` 폴더 전체를 `code/yahboomcar_ros2_ws/src/` 로 복사
4. `colcon build` 테스트 (PC나 로버)
5. git add & commit & push

이제부터 이 코드도 git 히스토리로 관리됩니다.

## 추가 팁

- `build/`, `install/`, `log/` 는 절대 git에 커밋하지 마세요 (.gitignore 처리됨)
- Maps, calibration 데이터는 경우에 따라 .gitignore 에 추가 고려
- VSCode로 개발할 때 "Remote - SSH" + "Dev Containers" 확장 추천
- 로버에서 VSCode로 작업하고 싶으면 컨테이너 attach 사용

---

이제 **PC에서 개발 → push → 로버 git pull** 사이클이 완성됩니다.
