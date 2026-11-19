// ros2_control SystemInterface for Yahboom ROSMASTER R2 (Ackermann).
// Talks to the STM32 expansion board directly over USB serial (no Rosmaster_Lib).
#pragma once

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "yahboom_r2_hardware/serial_port.hpp"
#include "yahboom_r2_hardware/yahboom_protocol.hpp"

namespace yahboom_r2_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class R2SystemHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum Role { TRACTION_RIGHT = 0, TRACTION_LEFT, STEER_RIGHT, STEER_LEFT, PASSIVE_RIGHT, PASSIVE_LEFT, N_ROLES };

  struct Joint
  {
    std::string name;
    double pos{0.0};
    double vel{0.0};
    double cmd{0.0};
    double sign{1.0};   // +1 if +joint rotation == forward / left turn
    bool present{false};
  };

  void send_stop();
  void update_states_from_report(double dt);

  rclcpp::Logger logger_{rclcpp::get_logger("R2SystemHardware")};
  rclcpp::Clock steady_{RCL_STEADY_TIME};

  // parameters
  std::string device_{"/dev/myserial"};
  int baud_{115200};
  double wheelbase_{0.23529};
  double traction_track_{0.1685};
  double steering_track_{0.13};
  double wheel_radius_{0.0345};
  double mcu_wheelbase_{0.23529};
  double max_linear_{1.0};
  double max_angular_{3.0};
  double max_steer_{0.6};
  double linear_scale_{1.0};
  double steer_report_scale_{0.90};
  double imu_gyro_sign_[3]{1.0, 1.0, -1.0};
  double imu_accel_sign_[3]{1.0, 1.0, 1.0};
  bool imu_zero_yaw_rate_when_stopped_{true};
  double command_period_s_{0.02};
  double rx_timeout_s_{0.5};
  bool enable_auto_report_{true};
  bool set_car_type_on_activate_{false};
  int encoder_ticks_per_rev_{0};
  int encoder_index_right_{1};
  int encoder_index_left_{0};
  double encoder_sign_right_{1.0};
  double encoder_sign_left_{1.0};

  std::array<Joint, N_ROLES> joints_{};

  // IMU sensor states: orientation xyzw, angular_velocity xyz, linear_acceleration xyz
  std::string imu_name_;
  std::array<double, 10> imu_{0, 0, 0, 1, 0, 0, 0, 0, 0, 0};

  // runtime
  SerialPort port_;
  proto::Parser parser_;
  proto::Report report_;
  rclcpp::Time last_rx_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_tx_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time activated_at_{0, 0, RCL_STEADY_TIME};
  double last_vx_cmd_{0.0};
  double last_wz_cmd_{0.0};
  bool enc_init_{false};
  std::array<int32_t, 4> enc_prev_{};
  double battery_v_{0.0};
};

}  // namespace yahboom_r2_hardware
