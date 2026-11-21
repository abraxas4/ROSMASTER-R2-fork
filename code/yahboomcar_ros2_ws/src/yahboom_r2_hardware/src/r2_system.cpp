#include "yahboom_r2_hardware/r2_system.hpp"

#include "yahboom_r2_hardware/serial_diag.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <map>
#include <stdexcept>
#include <unistd.h>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace yahboom_r2_hardware
{
namespace
{
double param_d(const std::unordered_map<std::string, std::string> & p, const std::string & k, double def)
{
  auto it = p.find(k);
  if (it == p.end() || it->second.empty()) {return def;}
  return std::stod(it->second);
}
int param_i(const std::unordered_map<std::string, std::string> & p, const std::string & k, int def)
{
  auto it = p.find(k);
  if (it == p.end() || it->second.empty()) {return def;}
  return std::stoi(it->second);
}
bool param_b(const std::unordered_map<std::string, std::string> & p, const std::string & k, bool def)
{
  auto it = p.find(k);
  if (it == p.end() || it->second.empty()) {return def;}
  std::string v = it->second;
  std::transform(v.begin(), v.end(), v.begin(), ::tolower);
  return v == "true" || v == "1";
}
std::string param_s(const std::unordered_map<std::string, std::string> & p, const std::string & k, const std::string & def)
{
  auto it = p.find(k);
  return (it == p.end() || it->second.empty()) ? def : it->second;
}
constexpr double kDeg2Rad = M_PI / 180.0;
}  // namespace

CallbackReturn R2SystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & p = info_.hardware_parameters;
  try {
    device_ = param_s(p, "device", device_);
    baud_ = param_i(p, "baud", baud_);
    wheelbase_ = param_d(p, "wheelbase", wheelbase_);
    traction_track_ = param_d(p, "traction_track", traction_track_);
    steering_track_ = param_d(p, "steering_track", steering_track_);
    wheel_radius_ = param_d(p, "wheel_radius", wheel_radius_);
    mcu_wheelbase_ = param_d(p, "mcu_wheelbase", wheelbase_);
    max_linear_ = param_d(p, "max_linear", max_linear_);
    max_angular_ = param_d(p, "max_angular", max_angular_);
    max_steer_ = param_d(p, "max_steer", max_steer_);
    linear_scale_ = param_d(p, "linear_scale", linear_scale_);
    steer_report_scale_ = param_d(p, "steer_report_scale", steer_report_scale_);
    imu_gyro_sign_[0] = param_d(p, "imu_gyro_x_sign", imu_gyro_sign_[0]);
    imu_gyro_sign_[1] = param_d(p, "imu_gyro_y_sign", imu_gyro_sign_[1]);
    imu_gyro_sign_[2] = param_d(p, "imu_gyro_z_sign", imu_gyro_sign_[2]);
    imu_accel_sign_[0] = param_d(p, "imu_accel_x_sign", imu_accel_sign_[0]);
    imu_accel_sign_[1] = param_d(p, "imu_accel_y_sign", imu_accel_sign_[1]);
    imu_accel_sign_[2] = param_d(p, "imu_accel_z_sign", imu_accel_sign_[2]);
    imu_zero_yaw_rate_when_stopped_ =
      param_b(p, "imu_zero_yaw_rate_when_stopped", imu_zero_yaw_rate_when_stopped_);
    command_period_s_ = param_d(p, "command_period_ms", command_period_s_ * 1000.0) / 1000.0;
    rx_timeout_s_ = param_d(p, "rx_timeout_ms", rx_timeout_s_ * 1000.0) / 1000.0;
    enable_auto_report_ = param_b(p, "enable_auto_report", enable_auto_report_);
    set_car_type_on_activate_ = param_b(p, "set_car_type_on_activate", set_car_type_on_activate_);
    encoder_ticks_per_rev_ = param_i(p, "encoder_ticks_per_rev", encoder_ticks_per_rev_);
    encoder_index_right_ = param_i(p, "encoder_index_right", encoder_index_right_);
    encoder_index_left_ = param_i(p, "encoder_index_left", encoder_index_left_);
    encoder_sign_right_ = param_d(p, "encoder_sign_right", encoder_sign_right_);
    encoder_sign_left_ = param_d(p, "encoder_sign_left", encoder_sign_left_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  const std::map<std::string, Role> roles{
    {"traction_right", TRACTION_RIGHT}, {"traction_left", TRACTION_LEFT},
    {"steer_right", STEER_RIGHT}, {"steer_left", STEER_LEFT},
    {"passive_right", PASSIVE_RIGHT}, {"passive_left", PASSIVE_LEFT}};

  for (const auto & j : info_.joints) {
    auto it = j.parameters.find("role");
    if (it == j.parameters.end() || roles.count(it->second) == 0) {
      RCLCPP_ERROR(logger_, "Joint '%s' needs <param name=\"role\"> (traction_right|traction_left|"
        "steer_right|steer_left|passive_right|passive_left)", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    auto & jt = joints_[roles.at(it->second)];
    jt.name = j.name;
    jt.present = true;
    jt.sign = param_d(j.parameters, "sign", 1.0);
  }
  for (int r : {TRACTION_RIGHT, TRACTION_LEFT, STEER_RIGHT, STEER_LEFT}) {
    if (!joints_[r].present) {
      RCLCPP_ERROR(logger_, "Missing joint role index %d (need 2 traction + 2 steering joints)", r);
      return CallbackReturn::ERROR;
    }
  }
  if (!info_.sensors.empty()) {
    imu_name_ = info_.sensors.front().name;
  }
  RCLCPP_INFO(logger_, "R2 hw: device=%s baud=%d L=%.4f Tt=%.4f Ts=%.4f r=%.4f imu=%s",
    device_.c_str(), baud_, wheelbase_, traction_track_, steering_track_, wheel_radius_,
    imu_name_.empty() ? "(none)" : imu_name_.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> R2SystemHardware::export_state_interfaces()
{
  namespace hi = hardware_interface;
  std::vector<hi::StateInterface> s;
  for (auto & j : joints_) {
    if (!j.present) {continue;}
    s.emplace_back(j.name, hi::HW_IF_POSITION, &j.pos);
    s.emplace_back(j.name, hi::HW_IF_VELOCITY, &j.vel);
  }
  if (!imu_name_.empty()) {
    static const char * names[10] = {
      "orientation.x", "orientation.y", "orientation.z", "orientation.w",
      "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
      "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
    for (size_t i = 0; i < 10; ++i) {
      s.emplace_back(imu_name_, names[i], &imu_[i]);
    }
  }
  return s;
}

std::vector<hardware_interface::CommandInterface> R2SystemHardware::export_command_interfaces()
{
  namespace hi = hardware_interface;
  std::vector<hi::CommandInterface> c;
  c.emplace_back(joints_[TRACTION_RIGHT].name, hi::HW_IF_VELOCITY, &joints_[TRACTION_RIGHT].cmd);
  c.emplace_back(joints_[TRACTION_LEFT].name, hi::HW_IF_VELOCITY, &joints_[TRACTION_LEFT].cmd);
  c.emplace_back(joints_[STEER_RIGHT].name, hi::HW_IF_POSITION, &joints_[STEER_RIGHT].cmd);
  c.emplace_back(joints_[STEER_LEFT].name, hi::HW_IF_POSITION, &joints_[STEER_LEFT].cmd);
  return c;
}

CallbackReturn R2SystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  log_serial_owners("on_configure before open");
  std::string err;
  if (!port_.open(device_, baud_, err)) {
    log_serial_owners("on_configure open FAILED");
    const std::string msg = "Cannot open " + device_ + ": " + err +
      " flag=" + driver_flag_state() +
      " owners=" + format_serial_owners(list_serial_owners(device_));
    RCLCPP_ERROR(logger_, "%s (is Ackman_driver_R2 / rosmaster_main.py / battery_osd holding the port?)",
      msg.c_str());
    diag_file_log("yahboom_r2_hardware", "ERROR", msg);
    diag_file_log("serial_owner", "ERROR", msg);
    return CallbackReturn::ERROR;
  }
  report_ = proto::Report{};
  enc_init_ = false;
  log_serial_owners("on_configure open OK");
  RCLCPP_INFO(logger_, "Opened %s @ %d", device_.c_str(), baud_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn R2SystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  log_serial_owners("on_cleanup before close");
  port_.close();
  log_serial_owners("on_cleanup after close");
  return CallbackReturn::SUCCESS;
}

CallbackReturn R2SystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  log_serial_owners("on_activate");
  if (!port_.is_open()) {
    const std::string msg = "on_activate but port is closed device=" + device_;
    RCLCPP_ERROR(logger_, "%s", msg.c_str());
    diag_file_log("yahboom_r2_hardware", "ERROR", msg);
    return CallbackReturn::ERROR;
  }
  if (set_car_type_on_activate_) {
    // Same as Ackman_driver_R2.py: car.set_car_type(5). Writes MCU flash.
    port_.write_all(proto::build_set_car_type(proto::CAR_TYPE_R2));
  }
  if (enable_auto_report_) {
    port_.write_all(proto::build_auto_report(true));
  }
  joints_[TRACTION_RIGHT].cmd = 0.0;
  joints_[TRACTION_LEFT].cmd = 0.0;
  joints_[STEER_RIGHT].cmd = 0.0;
  joints_[STEER_LEFT].cmd = 0.0;
  send_stop();
  activated_at_ = steady_.now();
  last_rx_ = activated_at_;
  RCLCPP_INFO(logger_, "R2 hardware active");
  return CallbackReturn::SUCCESS;
}

CallbackReturn R2SystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  log_serial_owners("on_deactivate");
  maybe_log_io_stats(true);
  send_stop();
  port_.write_all(proto::build_reset_state());  // Rosmaster_Lib.reset_car_state()
  return CallbackReturn::SUCCESS;
}

void R2SystemHardware::send_stop()
{
  port_.write_all(proto::build_motion(proto::CAR_TYPE_R2, 0.0, 0.0, 0.0));
  last_vx_cmd_ = 0.0;
  last_wz_cmd_ = 0.0;
  last_tx_ = steady_.now();
}

hardware_interface::return_type R2SystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  uint8_t buf[512];
  size_t frames = 0;
  for (int i = 0; i < 8; ++i) {
    const long n = port_.read_some(buf, sizeof(buf));
    if (n < 0) {
      ++read_errors_;
      const std::string msg = "Serial read error on " + device_ +
        " errno=" + std::to_string(port_.last_errno()) + " " +
        (port_.last_errno() ? std::strerror(port_.last_errno()) : "") +
        " owners=" + format_serial_owners(list_serial_owners(device_));
      RCLCPP_ERROR(logger_, "%s", msg.c_str());
      maybe_log_io_stats(true);
      return hardware_interface::return_type::ERROR;
    }
    if (n == 0) {break;}
    frames += parser_.feed(buf, static_cast<size_t>(n), report_);
  }
  const auto now = steady_.now();
  if (frames > 0) {
    last_rx_ = now;
  } else if ((now - last_rx_).seconds() > rx_timeout_s_) {
    ++timeout_events_;
    RCLCPP_ERROR(logger_, "No MCU report for %.0f ms (auto report off or cable unplugged)",
      (now - last_rx_).seconds() * 1000.0);
    maybe_log_io_stats(true);
    return hardware_interface::return_type::ERROR;
  }
  update_states_from_report(period.seconds());
  maybe_log_io_stats(false);
  return hardware_interface::return_type::OK;
}

void R2SystemHardware::update_states_from_report(double dt)
{
  // ---- body state from FUNC_REPORT_SPEED (0x0A), as Ackman_driver_R2.pub_data + base_node_R2 use it
  const double vx = report_.vx * linear_scale_;
  // R2: vy*1000 = steering angle [deg], + = left (base_node_R2: R = L / tan(deg), scale 0.90)
  const double delta = report_.vy * 1000.0 * steer_report_scale_ * kDeg2Rad;
  const double t = std::tan(delta);
  battery_v_ = report_.battery_v;
  {
    // battery voltage for user-space scripts (no extra ROS deps): /dev/shm/r2_battery_v, ~1 Hz
    static double batt_acc = 1.0;
    batt_acc += dt;
    if (batt_acc >= 1.0 && battery_v_ > 0.0) {
      batt_acc = 0.0;
      std::ofstream f("/dev/shm/r2_battery_v.tmp", std::ios::trunc);
      if (f) {f << battery_v_ << "\n"; f.close(); std::rename("/dev/shm/r2_battery_v.tmp", "/dev/shm/r2_battery_v");}
    }
  }

  // Raw MCU encoder counts (FUNC_REPORT_ENCODER 0x0D) for later logging.
  // Does not affect odometry while encoder_ticks_per_rev == 0.
  if (report_.has_encoder) {
    static double enc_acc = 0.1;
    enc_acc += dt;
    if (enc_acc >= 0.1) {
      enc_acc = 0.0;
      std::ofstream f("/dev/shm/r2_encoder.tmp", std::ios::trunc);
      if (f) {
        f << report_.encoder[0] << ' ' << report_.encoder[1] << ' '
          << report_.encoder[2] << ' ' << report_.encoder[3] << "\n";
        f.close();
        std::rename("/dev/shm/r2_encoder.tmp", "/dev/shm/r2_encoder");
      }
    }
  }

  // traction wheels (rear, fixed axle): v_side = vx * (1 -/+ (T/2)*tan(delta)/L)
  double w_right = vx * (1.0 + 0.5 * traction_track_ * t / wheelbase_) / wheel_radius_;
  double w_left = vx * (1.0 - 0.5 * traction_track_ * t / wheelbase_) / wheel_radius_;

  if (encoder_ticks_per_rev_ > 0 && report_.has_encoder && dt > 0.0) {
    if (enc_init_) {
      const double k = 2.0 * M_PI / encoder_ticks_per_rev_;
      w_right = encoder_sign_right_ * (report_.encoder[encoder_index_right_] - enc_prev_[encoder_index_right_]) * k / dt;
      w_left = encoder_sign_left_ * (report_.encoder[encoder_index_left_] - enc_prev_[encoder_index_left_]) * k / dt;
    }
    enc_prev_ = report_.encoder;
    enc_init_ = true;
  }

  auto & jr = joints_[TRACTION_RIGHT];
  auto & jl = joints_[TRACTION_LEFT];
  jr.vel = jr.sign * w_right;
  jl.vel = jl.sign * w_left;
  jr.pos += jr.vel * dt;
  jl.pos += jl.vel * dt;

  // steering joints: Ackermann inner/outer from bicycle angle
  const double ts2 = 0.5 * steering_track_;
  const double dl = std::atan2(wheelbase_ * t, wheelbase_ - ts2 * t);
  const double dr = std::atan2(wheelbase_ * t, wheelbase_ + ts2 * t);
  auto & sr = joints_[STEER_RIGHT];
  auto & sl = joints_[STEER_LEFT];
  const double sr_new = sr.sign * (std::abs(t) < 1e-9 ? 0.0 : dr);
  const double sl_new = sl.sign * (std::abs(t) < 1e-9 ? 0.0 : dl);
  sr.vel = dt > 0.0 ? (sr_new - sr.pos) / dt : 0.0;
  sl.vel = dt > 0.0 ? (sl_new - sl.pos) / dt : 0.0;
  sr.pos = sr_new;
  sl.pos = sl_new;

  // passive front wheels (display only): v = vx / cos(delta_side)
  if (joints_[PASSIVE_RIGHT].present) {
    auto & p = joints_[PASSIVE_RIGHT];
    p.vel = p.sign * vx * (1.0 + ts2 * t / wheelbase_) / std::max(std::cos(dr), 0.2) / wheel_radius_;
    p.pos += p.vel * dt;
  }
  if (joints_[PASSIVE_LEFT].present) {
    auto & p = joints_[PASSIVE_LEFT];
    p.vel = p.sign * vx * (1.0 - ts2 * t / wheelbase_) / std::max(std::cos(dl), 0.2) / wheel_radius_;
    p.pos += p.vel * dt;
  }

  // ---- IMU (0x0B MPU9250 or 0x0E ICM20948), signs as Ackman_driver_R2.pub_data (gz flipped)
  if (!imu_name_.empty() && report_.has_imu) {
    for (int i = 0; i < 3; ++i) {
      imu_[4 + i] = imu_gyro_sign_[i] * report_.gyro[i];
      imu_[7 + i] = imu_accel_sign_[i] * report_.accel[i];
    }
    if (imu_zero_yaw_rate_when_stopped_ && std::abs(vx) < 0.03) {
      imu_[6] = 0.0;  // Ackman_driver_R2: hold yaw while stopped
    }
    if (report_.has_att) {
      const double cr = std::cos(report_.roll / 2), sr2 = std::sin(report_.roll / 2);
      const double cp = std::cos(report_.pitch / 2), sp = std::sin(report_.pitch / 2);
      const double cy = std::cos(report_.yaw / 2), sy = std::sin(report_.yaw / 2);
      imu_[0] = sr2 * cp * cy - cr * sp * sy;
      imu_[1] = cr * sp * cy + sr2 * cp * sy;
      imu_[2] = cr * cp * sy - sr2 * sp * cy;
      imu_[3] = cr * cp * cy + sr2 * sp * sy;
    }
  }
}

hardware_interface::return_type R2SystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto & jr = joints_[TRACTION_RIGHT];
  const auto & jl = joints_[TRACTION_LEFT];
  const auto & sr = joints_[STEER_RIGHT];
  const auto & sl = joints_[STEER_LEFT];

  double wr = jr.sign * jr.cmd, wl = jl.sign * jl.cmd;
  double dr = sr.sign * sr.cmd, dl = sl.sign * sl.cmd;
  if (!std::isfinite(wr) || !std::isfinite(wl)) {wr = wl = 0.0;}
  if (!std::isfinite(dr) || !std::isfinite(dl)) {dr = dl = 0.0;}

  // body speed from the two rear wheels
  double vx = 0.5 * (wr + wl) * wheel_radius_;
  // bicycle-equivalent steering: cot(delta) = (cot(dl) + cot(dr)) / 2  ->  tan(delta) = 2 tl tr / (tl + tr)
  const double tl = std::tan(std::clamp(dl, -max_steer_, max_steer_));
  const double tr = std::tan(std::clamp(dr, -max_steer_, max_steer_));
  double tan_delta = 0.0;
  if (std::abs(tl + tr) > 1e-9 && tl * tr > 0.0) {
    tan_delta = 2.0 * tl * tr / (tl + tr);
  } else {
    tan_delta = 0.5 * (tl + tr);
  }
  vx = std::clamp(vx, -max_linear_, max_linear_);
  // Rosmaster set_car_motion(vx, 0, wz): R2 firmware turns wz into a servo angle using its own wheelbase.
  double wz = vx * tan_delta / mcu_wheelbase_;
  wz = std::clamp(wz, -max_angular_, max_angular_);

  const auto now = steady_.now();
  const double since = (now - last_tx_).seconds();
  const bool changed = std::abs(vx - last_vx_cmd_) > 1e-3 || std::abs(wz - last_wz_cmd_) > 1e-3;
  // keepalive every command_period; on change at most every 5 ms (13-byte frame on 115200 baud)
  if (since >= command_period_s_ || (changed && since >= 0.005)) {
    if (!port_.write_all(proto::build_motion(proto::CAR_TYPE_R2, vx, 0.0, wz))) {
      ++write_errors_;
      const std::string msg = std::string("Serial write failed errno=") +
        std::to_string(port_.last_errno()) + " " +
        (port_.last_errno() ? std::strerror(port_.last_errno()) : "") +
        " owners=" + format_serial_owners(list_serial_owners(device_));
      RCLCPP_ERROR(logger_, "%s", msg.c_str());
      maybe_log_io_stats(true);
      return hardware_interface::return_type::ERROR;
    }
    last_vx_cmd_ = vx;
    last_wz_cmd_ = wz;
    last_tx_ = now;
  }
  return hardware_interface::return_type::OK;
}

void R2SystemHardware::log_serial_owners(const char * when)
{
  const int self = ::getpid();
  const auto owners = list_serial_owners(device_);
  const std::string others = format_other_owners(owners, self);
  const std::string msg = std::string(when) + " device=" + device_ +
    " self_pid=" + std::to_string(self) +
    " flag=" + driver_flag_state() +
    " (plugin does not create/remove " + std::string("/tmp/r2_serial_for_driver") + ")" +
    " owners=" + format_serial_owners(owners) +
    " others=" + others;
  diag_file_log("yahboom_r2_hardware", others == "none" ? "INFO" : "WARN", msg);
  diag_file_log("serial_owner", others == "none" ? "INFO" : "WARN", msg);
  if (others != "none") {
    RCLCPP_WARN(logger_, "Serial %s also held by other process(es): %s (%s)",
      device_.c_str(), others.c_str(), when);
  } else {
    RCLCPP_INFO(logger_, "%s", msg.c_str());
  }
}

void R2SystemHardware::maybe_log_io_stats(bool force)
{
  const auto now = steady_.now();
  const bool due = force ||
    (last_io_diag_.nanoseconds() == 0) ||
    ((now - last_io_diag_).seconds() >= 1.0);
  if (!due) {return;}
  const size_t cs = parser_.checksum_errors();
  const size_t hd = parser_.header_errors();
  const bool changed =
    cs != last_logged_checksum_ ||
    hd != last_logged_header_ ||
    read_errors_ != last_logged_read_errors_ ||
    write_errors_ != last_logged_write_errors_ ||
    timeout_events_ != last_logged_timeouts_;
  if (!changed && !force) {return;}
  last_io_diag_ = now;
  const std::string msg =
    "serial io checksum_err=" + std::to_string(cs) +
    " header_err=" + std::to_string(hd) +
    " read_err=" + std::to_string(read_errors_) +
    " write_err=" + std::to_string(write_errors_) +
    " rx_timeout=" + std::to_string(timeout_events_) +
    " flag=" + driver_flag_state();
  diag_file_log("yahboom_r2_hardware", changed ? "WARN" : "INFO", msg);
  last_logged_checksum_ = cs;
  last_logged_header_ = hd;
  last_logged_read_errors_ = read_errors_;
  last_logged_write_errors_ = write_errors_;
  last_logged_timeouts_ = timeout_events_;
}

}  // namespace yahboom_r2_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yahboom_r2_hardware::R2SystemHardware, hardware_interface::SystemInterface)
