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
 * using the push-style binary protocol in serial_protocol.h. The firmware
 * streams a telemetry packet (measured wheel speed, raw per-wheel encoder
 * ticks, battery/temperature) every time it gets a fresh reading from the
 * hoverboard; there is no request/response. Per-wheel joint position is
 * built up here from the raw encoder ticks (see read()), and vehicle
 * odometry (x, y, theta) is left to diff_drive_controller rather than
 * trusting an onboard pose estimate.
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
  // reply (the firmware never acknowledges commands).
  bool sendCommand(int16_t speed_left, int16_t speed_right);

  // Passively waits (up to timeout) for one valid telemetry packet, used to
  // confirm the link is alive on configure/activate. There is no request to
  // send -- the firmware only speaks when it has something to say.
  bool waitForTelemetry(std::chrono::milliseconds timeout);

  // Applies one decoded telemetry packet to wheel_positions_/velocities_.
  void applyTelemetry(const protocol::TelemetryPacket & pkt, const rclcpp::Duration & period);

  // Signed tick delta accounting for wrap-around at encoder_max_, matching
  // the firmware's own Odom::wrapDelta().
  int wrapTickDelta(int current, int previous) const;

  // Serial communication
  std::string serial_port_;
  int baud_rate_;
  int serial_fd_{-1};
  protocol::TelemetryParser telemetry_parser_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::chrono::steady_clock::time_point last_telemetry_time_;
  static constexpr std::chrono::milliseconds kTelemetryTimeout{500};

  // Wrap-aware encoder tick tracking (see wrapTickDelta()/applyTelemetry()).
  bool have_last_ticks_{false};
  int last_tick_l_{0};
  int last_tick_r_{0};

  // Wheel joint states
  std::vector<double> wheel_positions_;
  std::vector<double> wheel_velocities_;

  // Wheel commands
  std::vector<double> wheel_velocity_commands_;

  // Parameters
  double wheel_radius_;         // meters; must match firmware's Odom wheel radius
  double wheel_separation_;     // meters; must match firmware's Odom wheel base
  double motor_command_scale_;  // raw hoverboard speed units per rad/s of wheel rotation
  int ticks_per_rev_;           // encoder ticks per wheel revolution; must match firmware's Odom
  int encoder_max_;             // encoder wrap modulus; must match firmware's Odom
};

}  // namespace uet_amr_hardware
