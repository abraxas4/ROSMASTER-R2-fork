// Diagnostic helpers: who holds a serial device, and rotating file logs.
// Logs go to $HOME/.ros/r2_logs/ (1 MiB x 5). No ROS dependency.
#pragma once

#include <string>
#include <vector>

namespace yahboom_r2_hardware
{

struct SerialOwner
{
  int pid{0};
  std::string comm;
  std::string exe;
  std::string fd;
  std::string fd_target;
};

/// Processes with an fd open on `device` (follows symlinks, compares inodes).
std::vector<SerialOwner> list_serial_owners(const std::string & device);

std::string format_serial_owners(const std::vector<SerialOwner> & owners);

/// Owners other than `self_pid` (pass getpid()).
std::string format_other_owners(const std::vector<SerialOwner> & owners, int self_pid);

bool driver_flag_present();
std::string driver_flag_state();  // "present age=...s" or "absent"

/// Append one line to ~/.ros/r2_logs/<which>.log with rotation (1 MiB x 5).
/// `which` is a stem such as "yahboom_r2_hardware" or "serial_owner".
void diag_file_log(const std::string & which, const std::string & level, const std::string & msg);

}  // namespace yahboom_r2_hardware
