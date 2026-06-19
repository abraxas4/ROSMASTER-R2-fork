# src/ - ROS2 Packages

이 폴더 안에 실제 ROS2 패키지들이 들어갑니다.

## 일반적인 구조 예시

```
src/
├── yahboom_rosmaster_driver/     # Yahboom 드라이버 (보통 Google Drive Code에 포함)
├── yahboom_rosmaster_bringup/
├── yahboom_rosmaster_description/
├── yahboom_rosmaster_navigation/
├── yahboom_rosmaster_vision/     # OpenCV, depth camera 등
├── ... (기타 패키지)
└── your_custom_package/
    ├── package.xml
    ├── setup.py
    └── ...
```

## Google Drive Code 복사 방법

Google Drive의 **5.Code** 폴더를 다운로드한 후:

1. 압축 해제
2. `yahboomcar_ros2_ws/src/` 안에 있는 모든 폴더를 이 위치로 복사
3. 이 README는 그대로 두고, 다른 파일은 덮어쓰기

## 개발 팁

- PC에서 개발할 때는 이 폴더를 VSCode로 열어서 작업
- 수정 후 git commit & push
- 로버에서 git pull 후 Docker 재시작 또는 colcon build 다시

이 폴더를 git으로 관리하면 "로버에서는 git pull만 하면 된다" 가 실현됩니다.
