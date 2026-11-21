#include "yahboom_r2_hardware/serial_diag.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <mutex>
#include <sstream>
#include <system_error>
#include <filesystem>

namespace yahboom_r2_hardware
{
namespace
{
constexpr const char * kFlagPath = "/tmp/r2_serial_for_driver";
constexpr std::uintmax_t kMaxBytes = 1u * 1024u * 1024u;
constexpr int kBackups = 5;

std::mutex g_log_mu;

std::string home_dir()
{
  const char * h = std::getenv("HOME");
  return (h && h[0]) ? std::string(h) : std::string("/tmp");
}

std::string log_dir() {return home_dir() + "/.ros/r2_logs";}

std::string read_text(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {return {};}
  std::string s;
  std::getline(in, s);
  return s;
}

std::string pid_comm(int pid)
{
  std::string s = read_text("/proc/" + std::to_string(pid) + "/comm");
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) {s.pop_back();}
  return s.empty() ? "?" : s;
}

std::string pid_exe(int pid)
{
  char buf[512];
  const ssize_t n = ::readlink(("/proc/" + std::to_string(pid) + "/exe").c_str(), buf, sizeof(buf) - 1);
  if (n <= 0) {return {};}
  buf[n] = 0;
  return std::string(buf);
}

bool same_inode(const std::string & a, const std::string & b)
{
  struct stat sa, sb;
  if (::stat(a.c_str(), &sa) != 0 || ::stat(b.c_str(), &sb) != 0) {return false;}
  return sa.st_dev == sb.st_dev && sa.st_ino == sb.st_ino;
}

void rotate_if_needed(const std::string & path)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::exists(path, ec)) {return;}
  const auto sz = fs::file_size(path, ec);
  if (ec || sz < kMaxBytes) {return;}
  fs::remove(path + "." + std::to_string(kBackups), ec);
  for (int i = kBackups - 1; i >= 1; --i) {
    const std::string src = path + "." + std::to_string(i);
    const std::string dst = path + "." + std::to_string(i + 1);
    if (fs::exists(src, ec)) {fs::rename(src, dst, ec);}
  }
  fs::rename(path, path + ".1", ec);
}

std::string now_stamp()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto t = system_clock::to_time_t(now);
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
  char out[80];
  std::snprintf(out, sizeof(out), "%s.%03d", buf, static_cast<int>(ms.count()));
  return out;
}
}  // namespace

std::vector<SerialOwner> list_serial_owners(const std::string & device)
{
  std::vector<SerialOwner> out;
  DIR * proc = ::opendir("/proc");
  if (!proc) {return out;}
  while (dirent * pe = ::readdir(proc)) {
    if (pe->d_type != DT_DIR && pe->d_type != DT_UNKNOWN) {continue;}
    char * end = nullptr;
    const long pidl = std::strtol(pe->d_name, &end, 10);
    if (!end || *end != 0 || pidl <= 0) {continue;}
    const int pid = static_cast<int>(pidl);
    const std::string fd_dir = "/proc/" + std::to_string(pid) + "/fd";
    DIR * fds = ::opendir(fd_dir.c_str());
    if (!fds) {continue;}
    while (dirent * fe = ::readdir(fds)) {
      if (fe->d_name[0] == '.') {continue;}
      const std::string fdp = fd_dir + "/" + fe->d_name;
      char buf[256];
      const ssize_t n = ::readlink(fdp.c_str(), buf, sizeof(buf) - 1);
      if (n <= 0) {continue;}
      buf[n] = 0;
      const std::string target(buf);
      if (target != device && !same_inode(fdp, device)) {continue;}
      SerialOwner o;
      o.pid = pid;
      o.comm = pid_comm(pid);
      o.exe = pid_exe(pid);
      o.fd = fe->d_name;
      o.fd_target = target;
      out.push_back(std::move(o));
    }
    ::closedir(fds);
  }
  ::closedir(proc);
  return out;
}

std::string format_serial_owners(const std::vector<SerialOwner> & owners)
{
  if (owners.empty()) {return "none";}
  std::ostringstream oss;
  for (size_t i = 0; i < owners.size(); ++i) {
    if (i) {oss << ", ";}
    const auto & o = owners[i];
    oss << o.pid << ':' << o.comm << "(fd=" << o.fd << "->" << o.fd_target << ')';
  }
  return oss.str();
}

std::string format_other_owners(const std::vector<SerialOwner> & owners, int self_pid)
{
  std::vector<SerialOwner> others;
  for (const auto & o : owners) {
    if (o.pid != self_pid) {others.push_back(o);}
  }
  return format_serial_owners(others);
}

bool driver_flag_present()
{
  struct stat st;
  return ::stat(kFlagPath, &st) == 0;
}

std::string driver_flag_state()
{
  struct stat st;
  if (::stat(kFlagPath, &st) != 0) {return std::string(kFlagPath) + " absent";}
  const double age = std::difftime(std::time(nullptr), st.st_mtime);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "%s present age=%.1fs", kFlagPath, age < 0 ? 0.0 : age);
  return buf;
}

void diag_file_log(const std::string & which, const std::string & level, const std::string & msg)
{
  std::lock_guard<std::mutex> lk(g_log_mu);
  namespace fs = std::filesystem;
  std::error_code ec;
  const std::string dir = log_dir();
  fs::create_directories(dir, ec);
  const std::string path = dir + "/" + which + ".log";
  rotate_if_needed(path);
  std::ofstream out(path, std::ios::app);
  if (!out) {return;}
  const int pid = ::getpid();
  out << now_stamp() << ' ' << which << '[' << pid << ':' << pid_comm(pid) << "] "
      << level << ' ' << msg << '\n';
}

}  // namespace yahboom_r2_hardware
