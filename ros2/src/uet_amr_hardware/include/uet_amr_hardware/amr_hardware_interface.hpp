// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License
 
#pragma once
 
#include <array>
#include <cstdint>
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
 
// ---------------------------------------------------------------------------
// Wire protocol -- MUST match protocol.h / protocol.cpp on the firmware side
// exactly (header bytes, command IDs, CRC8 poly/init, max data length).
//
// TODO: the numeric values below are PLACEHOLDERS. Copy the real values from
// your firmware's protocol.h before building, or the firmware and this node
// will silently fail to understand each other (bad header byte / bad CRC ->
// every packet gets dropped, with no obvious error on either side).
// ---------------------------------------------------------------------------
namespace protocol
{
constexpr uint8_t HEADER_MASTER_TO_SLAVE = 0xAA;  // TODO: verify vs PROTOCOL_HEADER_MASTER_TO_SLAVE
constexpr uint8_t HEADER_SLAVE_TO_MASTER = 0xBB;  // TODO: verify vs PROTOCOL_HEADER_SLAVE_TO_MASTER
constexpr uint8_t MAX_DATA_LEN = 32;              // TODO: verify vs PROTOCOL_MAX_DATA_LEN
 
enum Cmd : uint8_t
{
  CMD_PING = 0x01,                // TODO: verify all IDs below against protocol.h
  CMD_START = 0x02,
  CMD_STOP = 0x03,
  CMD_RESET_ENCODER = 0x04,
  CMD_RESET_ODOMETRY = 0x05,
  CMD_SET_VELOCITY = 0x10,
  CMD_SET_SMOOTH_VELOCITY = 0x11,
  CMD_ODOMETRY = 0x20,
  CMD_VELOCITY = 0x21,
  CMD_ENCODER_POS = 0x22,
  CMD_STATUS = 0x23,
  CMD_ACK_PING = 0x30,
  CMD_ACK_CONTROL = 0x31,
  CMD_WATCHDOG = 0x32,
};
}  // namespace protocol
 
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
  // --- Serial helpers ---
  bool openSerialPort();
  void closeSerialPort();
  bool sendPacket(uint8_t cmd, const uint8_t * data, uint8_t len);
  // Drains everything currently waiting on the fd and feeds it through the
  // packet parser; returns true if at least one full, CRC-valid packet was
  // decoded (result left in last_cmd_/last_data_/last_len_).
  bool pollIncoming();
  void feedByte(uint8_t b);
  static uint8_t crc8(uint8_t cmd, uint8_t len, const uint8_t * data);
 
  // Non-blocking byte-at-a-time parser state (mirrors Protocol::feedByte on
  // the firmware side: WAIT_HEADER -> WAIT_CMD -> WAIT_LEN -> WAIT_DATA -> WAIT_CRC)
  enum class ParseState { WAIT_HEADER, WAIT_CMD, WAIT_LEN, WAIT_DATA, WAIT_CRC } parse_state_{ParseState::WAIT_HEADER};
  uint8_t rx_cmd_{0};
  uint8_t rx_len_{0};
  uint8_t rx_index_{0};
  std::array<uint8_t, protocol::MAX_DATA_LEN> rx_data_{};
 
  // Most recently decoded packet, consumed by read()
  uint8_t last_cmd_{0};
  uint8_t last_len_{0};
  std::array<uint8_t, protocol::MAX_DATA_LEN> last_data_{};
 
  // Serial communication
  std::string serial_port_;
  int baud_rate_;
  int serial_fd_{-1};
 
  // Wheel joint states
  std::vector<double> wheel_positions_;
  std::vector<double> wheel_velocities_;
 
  // Wheel commands
  std::vector<double> wheel_velocity_commands_;
  std::vector<double> last_sent_commands_;  // avoid re-sending identical commands every cycle
 
  // Parameters
  double wheel_radius_;
  double wheel_separation_;
  double encoder_ticks_per_rev_;
};
 
}  // namespace uet_amr_hardware