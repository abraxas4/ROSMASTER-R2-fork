#include "yahboom_r2_hardware/yahboom_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace yahboom_r2_hardware
{
namespace proto
{
namespace
{
int16_t to_i16(double v)
{
  const double c = std::clamp(v, -32768.0, 32767.0);
  return static_cast<int16_t>(std::lround(std::trunc(c)));
}
void push_i16(std::vector<uint8_t> & v, int16_t x)
{
  const uint16_t u = static_cast<uint16_t>(x);
  v.push_back(static_cast<uint8_t>(u & 0xFF));
  v.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
}
int16_t rd_i16(const std::vector<uint8_t> & d, size_t o)
{
  return static_cast<int16_t>(static_cast<uint16_t>(d[o]) | (static_cast<uint16_t>(d[o + 1]) << 8));
}
int32_t rd_i32(const std::vector<uint8_t> & d, size_t o)
{
  uint32_t u = 0;
  for (size_t i = 0; i < 4; ++i) {u |= static_cast<uint32_t>(d[o + i]) << (8 * i);}
  return static_cast<int32_t>(u);
}
}  // namespace

std::vector<uint8_t> build_frame(uint8_t func, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> cmd{HEAD, DEVICE_ID, 0x00, func};
  cmd.insert(cmd.end(), payload.begin(), payload.end());
  cmd[2] = static_cast<uint8_t>(cmd.size() - 1);  // lib: cmd[2] = len(cmd) - 1
  unsigned sum = COMPLEMENT;
  for (auto b : cmd) {sum += b;}
  cmd.push_back(static_cast<uint8_t>(sum & 0xFF));
  return cmd;
}

std::vector<uint8_t> build_motion(uint8_t car_type, double vx, double vy, double vz)
{
  // lib: int(v*1000) -> truncation toward zero, then struct.pack('h')
  std::vector<uint8_t> p{car_type};
  push_i16(p, to_i16(vx * 1000.0));
  push_i16(p, to_i16(vy * 1000.0));
  push_i16(p, to_i16(vz * 1000.0));
  return build_frame(FUNC_MOTION, p);
}

std::vector<uint8_t> build_auto_report(bool enable)
{
  // lib uses a fixed len byte 0x05 here; build_frame yields the same value (6 bytes - 1).
  return build_frame(FUNC_AUTO_REPORT, {static_cast<uint8_t>(enable ? 1 : 0), 0x00});
}

std::vector<uint8_t> build_reset_state()
{
  // lib: [0xFF, 0xFC, 0x04, 0x0F, 0x5F, cs] -> same as build_frame
  return build_frame(FUNC_RESET_STATE, {0x5F});
}

std::vector<uint8_t> build_set_car_type(uint8_t car_type)
{
  return build_frame(FUNC_SET_CAR_TYPE, {car_type, 0x5F});
}

size_t Parser::feed(const uint8_t * data, size_t n, Report & out)
{
  size_t frames = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t b = data[i];
    switch (st_) {
      case St::HEAD1:
        if (b == HEAD) {st_ = St::HEAD2;}
        break;
      case St::HEAD2:
        if (b == DEVICE_ID_REPORT) {
          st_ = St::LEN;
        } else if (b == HEAD) {
          st_ = St::HEAD2;
        } else {
          ++header_errors_;
          st_ = St::HEAD1;
        }
        break;
      case St::LEN:
        len_ = b;
        if (len_ >= 3 && len_ < 64) {
          st_ = St::TYPE;
        } else {
          ++header_errors_;
          st_ = St::HEAD1;
        }
        break;
      case St::TYPE:
        type_ = b;
        buf_.clear();
        st_ = St::DATA;
        break;
      case St::DATA:
        buf_.push_back(b);
        if (buf_.size() == static_cast<size_t>(len_ - 2)) {
          unsigned sum = static_cast<unsigned>(len_) + type_;
          for (size_t k = 0; k + 1 < buf_.size(); ++k) {sum += buf_[k];}
          if ((sum & 0xFF) == buf_.back()) {
            buf_.pop_back();
            decode(type_, buf_, out);
            ++frames;
          } else {
            ++checksum_errors_;
          }
          st_ = St::HEAD1;
        }
        break;
    }
  }
  return frames;
}

void Parser::decode(uint8_t type, const std::vector<uint8_t> & d, Report & out)
{
  // Scale factors and sign conventions copied from Rosmaster_Lib.__parse_data
  constexpr double kGyroMpu = 1.0 / 3754.9;   // +-500 dps
  constexpr double kAccMpu = 1.0 / 1671.84;   // +-2 g
  constexpr double kMagMpu = 1.0;
  constexpr double kGyroIcm = 1.0 / 1000.0;  // ICM20948 frame is pre-scaled by the MCU
  constexpr double kAccIcm = 1.0 / 1000.0;
  constexpr double kMagIcm = 1.0 / 1000.0;

  switch (type) {
    case FUNC_REPORT_SPEED:
      if (d.size() >= 7) {
        out.vx = rd_i16(d, 0) / 1000.0;
        out.vy = rd_i16(d, 2) / 1000.0;
        out.vz = rd_i16(d, 4) / 1000.0;
        out.battery_v = d[6] / 10.0;
        out.has_speed = true;
      }
      break;
    case FUNC_REPORT_MPU_RAW:
      if (d.size() >= 18) {
        out.gyro = {rd_i16(d, 0) * kGyroMpu, rd_i16(d, 2) * -kGyroMpu, rd_i16(d, 4) * -kGyroMpu};
        out.accel = {rd_i16(d, 6) * kAccMpu, rd_i16(d, 8) * kAccMpu, rd_i16(d, 10) * kAccMpu};
        out.mag = {rd_i16(d, 12) * kMagMpu, rd_i16(d, 14) * kMagMpu, rd_i16(d, 16) * kMagMpu};
        out.has_imu = true;
      }
      break;
    case FUNC_REPORT_ICM_RAW:
      if (d.size() >= 18) {
        out.gyro = {rd_i16(d, 0) * kGyroIcm, rd_i16(d, 2) * kGyroIcm, rd_i16(d, 4) * kGyroIcm};
        out.accel = {rd_i16(d, 6) * kAccIcm, rd_i16(d, 8) * kAccIcm, rd_i16(d, 10) * kAccIcm};
        out.mag = {rd_i16(d, 12) * kMagIcm, rd_i16(d, 14) * kMagIcm, rd_i16(d, 16) * kMagIcm};
        out.has_imu = true;
      }
      break;
    case FUNC_REPORT_IMU_ATT:
      if (d.size() >= 6) {
        out.roll = rd_i16(d, 0) / 10000.0;
        out.pitch = rd_i16(d, 2) / 10000.0;
        out.yaw = rd_i16(d, 4) / 10000.0;
        out.has_att = true;
      }
      break;
    case FUNC_REPORT_ENCODER:
      if (d.size() >= 16) {
        for (size_t k = 0; k < 4; ++k) {out.encoder[k] = rd_i32(d, 4 * k);}
        out.has_encoder = true;
      }
      break;
    default:
      break;
  }
}

}  // namespace proto
}  // namespace yahboom_r2_hardware
