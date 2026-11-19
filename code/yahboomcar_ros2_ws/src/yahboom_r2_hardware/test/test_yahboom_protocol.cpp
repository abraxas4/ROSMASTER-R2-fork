// Expected byte strings were generated with the exact expressions of Rosmaster_Lib.py
// (set_car_motion / reset_car_state / set_auto_report_state / __receive_data).
#include <gtest/gtest.h>

#include <vector>

#include "yahboom_r2_hardware/yahboom_protocol.hpp"

using namespace yahboom_r2_hardware::proto;

TEST(Protocol, MotionFrameMatchesRosmasterLib)
{
  EXPECT_EQ(build_motion(CAR_TYPE_R2, 0.3, 0.0, 1.2),
    (std::vector<uint8_t>{0xFF, 0xFC, 0x0A, 0x12, 0x05, 0x2C, 0x01, 0x00, 0x00, 0xB0, 0x04, 0x02}));
  EXPECT_EQ(build_motion(CAR_TYPE_R2, -0.25, 0.0, -0.7),
    (std::vector<uint8_t>{0xFF, 0xFC, 0x0A, 0x12, 0x05, 0x06, 0xFF, 0x00, 0x00, 0x44, 0xFD, 0x67}));
}

TEST(Protocol, ResetAndAutoReport)
{
  EXPECT_EQ(build_reset_state(), (std::vector<uint8_t>{0xFF, 0xFC, 0x04, 0x0F, 0x5F, 0x72}));
  EXPECT_EQ(build_auto_report(true), (std::vector<uint8_t>{0xFF, 0xFC, 0x05, 0x01, 0x01, 0x00, 0x07}));
}

TEST(Protocol, ParseSpeedReportWithNoiseAndSplit)
{
  // vx = 0.512 m/s, vy = -0.012 (R2: -12 deg steering), battery 11.8 V
  // len byte = payload(7) + 3, as the MCU sends it (lib: data_len = len - 2 incl. checksum)
  const std::vector<uint8_t> f{0xFF, 0xFB, 0x0A, 0x0A, 0x00, 0x02, 0xF4, 0xFF, 0x00, 0x00, 0x76, 0x7F};
  std::vector<uint8_t> stream{0x00, 0xFF, 0x13};
  stream.insert(stream.end(), f.begin(), f.end());
  Parser p;
  Report r;
  size_t n = p.feed(stream.data(), 7, r);
  n += p.feed(stream.data() + 7, stream.size() - 7, r);
  EXPECT_EQ(n, 1u);
  EXPECT_TRUE(r.has_speed);
  EXPECT_NEAR(r.vx, 0.512, 1e-9);
  EXPECT_NEAR(r.vy * 1000.0, -12.0, 1e-9);
  EXPECT_NEAR(r.battery_v, 11.8, 1e-9);
}

TEST(Protocol, BadChecksumRejected)
{
  std::vector<uint8_t> f{0xFF, 0xFB, 0x0A, 0x0A, 0x00, 0x02, 0xF4, 0xFF, 0x00, 0x00, 0x76, 0x80};
  Parser p;
  Report r;
  EXPECT_EQ(p.feed(f.data(), f.size(), r), 0u);
  EXPECT_FALSE(r.has_speed);
  EXPECT_EQ(p.checksum_errors(), 1u);
}
