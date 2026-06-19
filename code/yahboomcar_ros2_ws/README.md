# yahboomcar_ros2_ws - ROS2 Workspace for ROSMASTER R2 (Jetson Orin)

이 폴더는 로버(Jetson Orin)의 Docker 컨테이너 내부에서 사용하는 실제 ROS2 워크스페이스입니다.

## 컨테이너 내부 경로
- Docker 안: `/root/yahboomcar_ros2_ws`

## 이 워크스페이스를 git으로 관리하는 방법 (추천 구조)

이 repo(`ROSMASTER-R2-fork`)의 `code/yahboomcar_ros2_ws/` 내용을 
로버의 호스트에서 아래 경로로 마운트해서 사용합니다.

**로버 호스트 추천 경로:**
```
/home/jetson/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws
```

그리고 `run_docker.sh` 에 아래 볼륨 마운트를 추가합니다:

```bash
-v /home/jetson/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws:/root/yahboomcar_ros2_ws \
```

이렇게 하면:
- PC에서 코드 수정 → commit & push (fork에)
- 로버에서 `git pull` → 코드가 바로 업데이트됨
- Docker 재시작하면 바로 반영

## Google Drive에서 Code 가져오기

1. Google Drive 링크에서 `5.Code` (또는 ROS2 관련) 다운로드
2. 압축 풀기
3. `yahboomcar_ros2_ws/` 안의 `src/` 폴더 내용을 이곳 `code/yahboomcar_ros2_ws/src/` 로 복사
4. 필요시 `package.xml`, `setup.py`, `CMakeLists.txt` 등 그대로 유지

## 빌드 방법 (컨테이너 안에서)

```bash
# 컨테이너 들어가기
cd /root/yahboomcar_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 주의사항
- `build/`, `install/`, `log/` 는 .gitignore 처리됨 (git에 올리지 마세요)
- 실제 하드웨어 의존적인 파일(launch의 device path 등)은 로버에서 조정
