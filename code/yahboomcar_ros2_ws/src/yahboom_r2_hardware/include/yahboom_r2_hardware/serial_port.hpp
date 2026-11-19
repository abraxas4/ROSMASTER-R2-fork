// Minimal POSIX serial port (8N1, raw, non-blocking). Linux only (Jetson).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace yahboom_r2_hardware
{
class SerialPort
{
public:
  ~SerialPort();
  bool open(const std::string & device, int baud, std::string & err);
  void close();
  bool is_open() const {return fd_ >= 0;}
  /// Non-blocking read of whatever is available. Returns bytes read (0 if none, -1 on error).
  long read_some(uint8_t * buf, size_t n);
  bool write_all(const std::vector<uint8_t> & data);

private:
  int fd_{-1};
};
}  // namespace yahboom_r2_hardware
