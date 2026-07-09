// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#pragma once

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace uet_amr_hardware
{

/**
 * @brief Hardware interface for UET AMR differential drive robot.
 *
 * Communicates with base controller firmware via serial (UART) using
 * a lightweight binary protocol. Exposes wheel position/velocity state
 * and velocity command interfaces to ros2_control.
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
  // Serial communication
  std::string serial_port_;
  int baud_rate_;
  int serial_fd_{-1};

  // Wheel joint states
  std::vector<double> wheel_positions_;
  std::vector<double> wheel_velocities_;

  // Wheel commands
  std::vector<double> wheel_velocity_commands_;

  // Parameters
  double wheel_radius_;
  double wheel_separation_;
  double encoder_ticks_per_rev_;
};

}  // namespace uet_amr_hardware
