#include "yahboom_r2_hardware/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace yahboom_r2_hardware
{
namespace
{
speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B0;
  }
}
}  // namespace

SerialPort::~SerialPort() {close();}

bool SerialPort::open(const std::string & device, int baud, std::string & err)
{
  close();
  const speed_t spd = to_speed(baud);
  if (spd == B0) {
    err = "unsupported baud rate";
    return false;
  }
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    err = std::string("open failed: ") + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    err = std::string("tcgetattr failed: ") + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    err = std::string("tcsetattr failed: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

long SerialPort::read_some(uint8_t * buf, size_t n)
{
  if (fd_ < 0) {return -1;}
  const ssize_t r = ::read(fd_, buf, n);
  if (r < 0) {
    return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
  }
  return static_cast<long>(r);
}

bool SerialPort::write_all(const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {return false;}
  size_t off = 0;
  int spins = 0;
  while (off < data.size()) {
    const ssize_t w = ::write(fd_, data.data() + off, data.size() - off);
    if (w < 0) {
      if ((errno == EAGAIN || errno == EWOULDBLOCK) && ++spins < 100) {continue;}
      return false;
    }
    off += static_cast<size_t>(w);
  }
  return true;
}
}  // namespace yahboom_r2_hardware
