// Yahboom ROSMASTER expansion board (STM32F103, YB-ERF01) serial protocol.
// Pure C++ (no ROS dependency) so it can be unit tested on any PC.
//
// Mirrors Rosmaster_Lib.py (py_install V3.3.9, mirror: github.com/Roblibs/Rosmaster_Lib):
//   host -> MCU : [0xFF, 0xFC, len, func, payload..., checksum]
//                 len      = number of bytes after the 0xFF 0xFC header minus 1 (= len(cmd)-1 in lib)
//                 checksum = (sum(all previous bytes) + 5) & 0xFF   (lib: sum(cmd, 257 - 0xFC))
//   MCU -> host : [0xFF, 0xFB, len, type, data..., checksum]
//                 len = payload + 3; bytes after type = len - 2 (payload + checksum)
//                 checksum = (len + type + sum(data without checksum)) & 0xFF   (lib: __receive_data)
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace yahboom_r2_hardware
{
namespace proto
{
constexpr uint8_t HEAD = 0xFF;
constexpr uint8_t DEVICE_ID = 0xFC;           // host -> MCU
constexpr uint8_t DEVICE_ID_REPORT = 0xFB;    // MCU -> host (DEVICE_ID - 1)
constexpr uint8_t COMPLEMENT = 257 - DEVICE_ID;  // = 5

// Function codes (Rosmaster_Lib.__init__)
constexpr uint8_t FUNC_AUTO_REPORT = 0x01;
constexpr uint8_t FUNC_BEEP = 0x02;
constexpr uint8_t FUNC_REPORT_SPEED = 0x0A;
constexpr uint8_t FUNC_REPORT_MPU_RAW = 0x0B;
constexpr uint8_t FUNC_REPORT_IMU_ATT = 0x0C;
constexpr uint8_t FUNC_REPORT_ENCODER = 0x0D;
constexpr uint8_t FUNC_REPORT_ICM_RAW = 0x0E;
constexpr uint8_t FUNC_RESET_STATE = 0x0F;
constexpr uint8_t FUNC_MOTION = 0x12;
constexpr uint8_t FUNC_SET_CAR_TYPE = 0x15;
constexpr uint8_t FUNC_AKM_STEER_ANGLE = 0x31;

constexpr uint8_t CAR_TYPE_R2 = 0x05;

/// Build a host->MCU frame. len byte and checksum are filled in like Rosmaster_Lib.
std::vector<uint8_t> build_frame(uint8_t func, const std::vector<uint8_t> & payload);

/// set_car_motion(v_x, v_y, v_z): FUNC_MOTION, car_type, int16 LE (value*1000) x3.
std::vector<uint8_t> build_motion(uint8_t car_type, double vx, double vy, double vz);

/// set_auto_report_state(enable, forever=False)
std::vector<uint8_t> build_auto_report(bool enable);

/// reset_car_state(): stop, LEDs off, buzzer off
std::vector<uint8_t> build_reset_state();

/// set_car_type(car_type): NOTE writes MCU flash (0x5F flag) like the Python driver does.
std::vector<uint8_t> build_set_car_type(uint8_t car_type);

struct Report
{
  // FUNC_REPORT_SPEED (0x0A)
  bool has_speed{false};
  double vx{0.0};       // m/s
  double vy{0.0};       // R2: steering angle in deg/1000 (see Ackman_driver_R2.py: vy*1000 = deg)
  double vz{0.0};       // R2: not valid (Ackman_driver_R2.py comment "this is invalued")
  double battery_v{0.0};

  // FUNC_REPORT_MPU_RAW (0x0B) / FUNC_REPORT_ICM_RAW (0x0E) - already scaled like the lib
  bool has_imu{false};
  std::array<double, 3> gyro{};   // rad/s
  std::array<double, 3> accel{};  // m/s^2
  std::array<double, 3> mag{};

  // FUNC_REPORT_IMU_ATT (0x0C)
  bool has_att{false};
  double roll{0.0}, pitch{0.0}, yaw{0.0};  // rad

  // FUNC_REPORT_ENCODER (0x0D)
  bool has_encoder{false};
  std::array<int32_t, 4> encoder{};
};

/// Incremental parser for the MCU->host byte stream (state machine of Rosmaster_Lib.__receive_data).
class Parser
{
public:
  /// Feed bytes. Returns number of valid frames decoded. Results are merged into `out`.
  size_t feed(const uint8_t * data, size_t n, Report & out);
  size_t checksum_errors() const {return checksum_errors_;}

private:
  void decode(uint8_t type, const std::vector<uint8_t> & data, Report & out);
  enum class St { HEAD1, HEAD2, LEN, TYPE, DATA } st_{St::HEAD1};
  uint8_t len_{0};
  uint8_t type_{0};
  std::vector<uint8_t> buf_;
  size_t checksum_errors_{0};
};

}  // namespace proto
}  // namespace yahboom_r2_hardware
