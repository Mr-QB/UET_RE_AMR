// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "uet_amr_hardware/protocol.hpp"

namespace uet_amr_hardware
{

/**
 * @brief Hardware interface for UET AMR differential drive robot.
 *
 * Communicates with the ESP32 base controller firmware via serial (UART)
 * using the fixed-size binary protocol in serial_protocol.h/.cpp. The
 * firmware exposes measured wheel speed and onboard (x, y, theta) odometry,
 * but no per-wheel encoder ticks -- wheel position is recovered by inverting
 * the same differential-drive kinematics the firmware used to build that
 * odometry from encoder deltas in the first place (see read()).
 *
 * Topics exposed (via hardware interface):
 *   - /joint_states (via ros2_control)
 *   - /odom (via diff_drive_controller)
 */
class AmrHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AmrHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool openSerial();
  void closeSerial();

  // Sends a command frame and returns immediately without waiting for a
  // reply (used for SetVelocity, which the firmware never acknowledges).
  bool sendCommand(protocol::Cmd cmd, int16_t speed_left, int16_t speed_right);

  // Sends a request frame (RequestVelocity/RequestOdometry/RequestStatus),
  // then blocks (up to timeout) reading/parsing replies until one with
  // cmd == expect_reply_cmd arrives.
  bool requestAndWait(
    protocol::Cmd request_cmd, protocol::Cmd expect_reply_cmd,
    std::vector<uint8_t> & out_payload, std::chrono::milliseconds timeout);

  // Serial communication
  std::string serial_port_;
  int baud_rate_;
  int serial_fd_{-1};
  protocol::FrameParser parser_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  int consecutive_comm_failures_{0};
  static constexpr int kMaxConsecutiveFailures = 20;  // ~200ms at 100Hz update rate

  // Last onboard odometry sample, used to recover per-wheel position deltas
  // (see read()). Firmware reports x/y in cm, theta in radians.
  bool have_last_pose_{false};
  double last_x_cm_{0.0};
  double last_y_cm_{0.0};
  double last_theta_rad_{0.0};

  // Wheel joint states
  std::vector<double> wheel_positions_;
  std::vector<double> wheel_velocities_;

  // Wheel commands
  std::vector<double> wheel_velocity_commands_;

  // Parameters
  double wheel_radius_;         // meters; must match firmware's Odom wheel radius
  double wheel_separation_;     // meters; must match firmware's Odom wheel base
  double motor_command_scale_;  // raw hoverboard speed units per rad/s of wheel rotation
};

}  // namespace uet_amr_hardware
