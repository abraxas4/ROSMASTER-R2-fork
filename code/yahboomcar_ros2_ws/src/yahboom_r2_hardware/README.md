# yahboom_r2_hardware

ROSMASTER R2 확장보드(STM32F103)를 ros2_control `SystemInterface`로 감싼 플러그인입니다. Python 라이브러리(Rosmaster_Lib) 없이 C++로 시리얼 프로토콜을 직접 처리합니다.

- 플러그인 이름: `yahboom_r2_hardware/R2SystemHardware`
- 파일:
  - `src/yahboom_protocol.cpp`: 프레임 생성·파싱 (ROS 의존 없음)
  - `src/serial_port.cpp`: POSIX 시리얼 (Linux 전용)
  - `src/r2_system.cpp`: ros2_control 인터페이스, 명령·상태 변환
  - `test/test_yahboom_protocol.cpp`: Rosmaster_Lib와 같은 바이트가 나오는지 확인하는 gtest
- 설정, 실행 방법, 한계, 2단계 계획은 `yahboom_r2_bringup/README.md`에 있습니다.

```bash
colcon test --packages-select yahboom_r2_hardware && colcon test-result --verbose
```
