// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#include "uet_amr_hardware/amr_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace uet_amr_hardware
{

hardware_interface::CallbackReturn AmrHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF ros2_control tag
  serial_port_  = info_.hardware_parameters.at("serial_port");
  baud_rate_    = std::stoi(info_.hardware_parameters.at("baud_rate"));
  wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
  encoder_ticks_per_rev_ = std::stod(info_.hardware_parameters.at("encoder_ticks_per_rev"));

  // Initialize state and command vectors (left, right)
  wheel_positions_.assign(2, 0.0);
  wheel_velocities_.assign(2, 0.0);
  wheel_velocity_commands_.assign(2, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("AmrHardwareInterface"),
    "Initialized: port=%s, baud=%d", serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Open serial port and verify communication with firmware
  RCLCPP_INFO(rclcpp::get_logger("AmrHardwareInterface"),
    "Configuring hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AmrHardwareInterface"), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AmrHardwareInterface"), "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AmrHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &wheel_positions_[0]);
  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &wheel_velocities_[0]);

  state_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_POSITION,
    &wheel_positions_[1]);
  state_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_VELOCITY,
    &wheel_velocities_[1]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AmrHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &wheel_velocity_commands_[0]);

  command_interfaces.emplace_back(
    info_.joints[1].name,
    hardware_interface::HW_IF_VELOCITY,
    &wheel_velocity_commands_[1]);

  return command_interfaces;
}

hardware_interface::return_type AmrHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // TODO: Read encoder data from firmware via serial port
  // Parse packet: [START_BYTE][LEFT_TICKS_4B][RIGHT_TICKS_4B][CHECKSUM]
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmrHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // TODO: Send velocity commands to firmware via serial port
  // Packet: [START_BYTE][LEFT_VEL_4B][RIGHT_VEL_4B][CHECKSUM]
  return hardware_interface::return_type::OK;
}

}  // namespace uet_amr_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  uet_amr_hardware::AmrHardwareInterface,
  hardware_interface::SystemInterface)
