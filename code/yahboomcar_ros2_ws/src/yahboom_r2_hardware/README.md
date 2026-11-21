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

## 엔코더 원값 (odometry에는 아직 쓰지 않음)

MCU가 `FUNC_REPORT_ENCODER`(0x0D)로 카운트를 보내면, 플러그인이 `/dev/shm/r2_encoder`에 네 개 정수(M1~M4)를 약 10 Hz로 기록합니다. `encoder_ticks_per_rev`는 기본값 0으로 두고, 바퀴 속도 계산은 기존처럼 MCU가 보고한 vx·조향각을 사용합니다.

나중에 엔코더로 속도를 쓰려면 (측정 후):

1. `/dev/shm/r2_encoder`가 갱신되는지 확인함
2. 좌·우 구동륜이 몇 번째 칸인지 확인한 뒤 `encoder_index_left` / `encoder_index_right`를 설정함
3. 한 바퀴당 틱 수를 재서 `encoder_ticks_per_rev`를 0보다 크게 넣고, 필요하면 `encoder_sign_left` / `encoder_sign_right`를 맞춤
4. 그러면 `update_states_from_report()`가 틱 차이로 바퀴 각속도를 계산함

