// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License
 
#include "uet_amr_hardware/amr_hardware_interface.hpp"
 
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
 
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>
 
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
 
namespace uet_amr_hardware
{
 
namespace
{
speed_t baudToTermios(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;  // fall back to the firmware's default (Serial.begin(115200))
  }
}
}  // namespace
 
// ---------------------------------------------------------------------------
// CRC8 -- identical polynomial/init to Protocol::crc8Update() in protocol.cpp.
// Both sides MUST compute the exact same value or every packet gets silently
// dropped (protocol.cpp's WAIT_CRC state resyncs on any mismatch without
// reporting an error).
// ---------------------------------------------------------------------------
uint8_t AmrHardwareInterface::crc8(uint8_t cmd, uint8_t len, const uint8_t * data)
{
  auto update = [](uint8_t crc, uint8_t b) -> uint8_t {
    crc ^= b;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
    return crc;
  };
  uint8_t crc = 0;
  crc = update(crc, cmd);
  crc = update(crc, len);
  for (uint8_t i = 0; i < len; i++) {
    crc = update(crc, data[i]);
  }
  return crc;
}
 
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
  last_sent_commands_.assign(2, std::nan(""));  // force the first write() to always send
 
  RCLCPP_INFO(rclcpp::get_logger("AmrHardwareInterface"),
    "Initialized: port=%s, baud=%d", serial_port_.c_str(), baud_rate_);
 
  return hardware_interface::CallbackReturn::SUCCESS;
}
 
bool AmrHardwareInterface::openSerialPort()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("AmrHardwareInterface"),
      "Failed to open %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }
 
  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("AmrHardwareInterface"),
      "tcgetattr failed: %s", std::strerror(errno));
    return false;
  }
 
  speed_t speed = baudToTermios(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
 
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 data bits
  tty.c_cflag &= ~PARENB;                       // no parity
  tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;                      // no hw flow control
  tty.c_cflag |= CREAD | CLOCAL;                // enable receiver, ignore modem lines
 
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // raw mode, no line buffering
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
 
  tty.c_cc[VMIN] = 0;   // non-blocking read: return immediately with whatever is available
  tty.c_cc[VTIME] = 0;
 
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("AmrHardwareInterface"),
      "tcsetattr failed: %s", std::strerror(errno));
    return false;
  }
 
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}
 
void AmrHardwareInterface::closeSerialPort()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}
 
bool AmrHardwareInterface::sendPacket(uint8_t cmd, const uint8_t * data, uint8_t len)
{
  if (serial_fd_ < 0) return false;
  uint8_t crc = crc8(cmd, len, data);
 
  std::vector<uint8_t> buf;
  buf.reserve(4 + len);
  buf.push_back(protocol::HEADER_MASTER_TO_SLAVE);
  buf.push_back(cmd);
  buf.push_back(len);
  if (len > 0 && data != nullptr) {
    buf.insert(buf.end(), data, data + len);
  }
  buf.push_back(crc);
 
  ssize_t written = ::write(serial_fd_, buf.data(), buf.size());
  return written == static_cast<ssize_t>(buf.size());
}
 
void AmrHardwareInterface::feedByte(uint8_t b)
{
  switch (parse_state_) {
    case ParseState::WAIT_HEADER:
      if (b == protocol::HEADER_SLAVE_TO_MASTER) {
        parse_state_ = ParseState::WAIT_CMD;
      }
      break;
 
    case ParseState::WAIT_CMD:
      rx_cmd_ = b;
      parse_state_ = ParseState::WAIT_LEN;
      break;
 
    case ParseState::WAIT_LEN:
      rx_len_ = b;
      if (rx_len_ > protocol::MAX_DATA_LEN) {
        parse_state_ = ParseState::WAIT_HEADER;  // corrupt length, resync
        break;
      }
      rx_index_ = 0;
      parse_state_ = (rx_len_ == 0) ? ParseState::WAIT_CRC : ParseState::WAIT_DATA;
      break;
 
    case ParseState::WAIT_DATA:
      rx_data_[rx_index_++] = b;
      if (rx_index_ >= rx_len_) {
        parse_state_ = ParseState::WAIT_CRC;
      }
      break;
 
    case ParseState::WAIT_CRC: {
      uint8_t expected = crc8(rx_cmd_, rx_len_, rx_data_.data());
      parse_state_ = ParseState::WAIT_HEADER;  // always resync, valid or not
      if (b == expected) {
        last_cmd_ = rx_cmd_;
        last_len_ = rx_len_;
        std::copy(rx_data_.begin(), rx_data_.begin() + rx_len_, last_data_.begin());
      }
      // else: CRC mismatch -- silently drop, same as firmware's protocol.cpp
      break;
    }
  }
}
 
bool AmrHardwareInterface::pollIncoming()
{
  if (serial_fd_ < 0) return false;
  uint8_t buf[256];
  bool got_packet = false;
  ssize_t n;
  while ((n = ::read(serial_fd_, buf, sizeof(buf))) > 0) {
    for (ssize_t i = 0; i < n; i++) {
      uint8_t prev_state = static_cast<uint8_t>(parse_state_);
      feedByte(buf[i]);
      // A full, CRC-valid packet was just decoded when we transition back to
      // WAIT_HEADER from WAIT_CRC in feedByte(); detect that transition here.
      if (prev_state == static_cast<uint8_t>(ParseState::WAIT_CRC) &&
          parse_state_ == ParseState::WAIT_HEADER)
      {
        got_packet = true;
      }
    }
  }
  return got_packet;
}
 
hardware_interface::CallbackReturn AmrHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AmrHardwareInterface"),
    "Configuring hardware interface, opening %s @ %d...", serial_port_.c_str(), baud_rate_);
 
  if (!openSerialPort()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
 
  // Give the Arduino a moment to finish its own boot/reset (many boards reset
  // on DTR toggle when the serial port is opened) before we start talking.
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
 
  if (!sendPacket(protocol::CMD_PING, nullptr, 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("AmrHardwareInterface"), "Ping failed to send.");
    return hardware_interface::CallbackReturn::ERROR;
  }
 
  return hardware_interface::CallbackReturn::SUCCESS;
}
 
hardware_interface::CallbackReturn AmrHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sendPacket(protocol::CMD_RESET_ENCODER, nullptr, 0);
  sendPacket(protocol::CMD_RESET_ODOMETRY, nullptr, 0);
  sendPacket(protocol::CMD_START, nullptr, 0);
  RCLCPP_INFO(rclcpp::get_logger("AmrHardwareInterface"), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}
 
hardware_interface::CallbackReturn AmrHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sendPacket(protocol::CMD_STOP, nullptr, 0);
  closeSerialPort();
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
  const rclcpp::Duration & period)
{
  // Ask for the latest encoder counts + measured velocity every cycle, then
  // drain and parse whatever has actually arrived (packets from a previous
  // cycle's request, most likely -- serial round-trip won't land same-cycle).
  sendPacket(protocol::CMD_ENCODER_POS, nullptr, 0);
  sendPacket(protocol::CMD_VELOCITY, nullptr, 0);
 
  while (pollIncoming()) {
    switch (last_cmd_) {
      case protocol::CMD_ENCODER_POS: {
        if (last_len_ < 8) break;
        int32_t left_ticks, right_ticks;
        std::memcpy(&left_ticks, &last_data_[0], 4);
        std::memcpy(&right_ticks, &last_data_[4], 4);
        // ticks -> radians: (ticks / ticks_per_rev) * 2*pi
        wheel_positions_[0] = (left_ticks / encoder_ticks_per_rev_) * 2.0 * M_PI;
        wheel_positions_[1] = (right_ticks / encoder_ticks_per_rev_) * 2.0 * M_PI;
        break;
      }
      case protocol::CMD_VELOCITY: {
        if (last_len_ < 8) break;
        float left_cm_s, right_cm_s;
        std::memcpy(&left_cm_s, &last_data_[0], 4);
        std::memcpy(&right_cm_s, &last_data_[4], 4);
        // firmware reports linear wheel-surface velocity in cm/s (see Encoder::readEncoder,
        // wheelRadius is in cm); ros2_control wants angular velocity in rad/s.
        double radius_cm = wheel_radius_ * 100.0;  // wheel_radius_ param assumed to be in meters
        wheel_velocities_[0] = (left_cm_s / radius_cm);
        wheel_velocities_[1] = (right_cm_s / radius_cm);
        break;
      }
      default:
        break;  // ACKs, STATUS, ODOMETRY, WATCHDOG -- not needed by read(), ignore
    }
  }
 
  return hardware_interface::return_type::OK;
}
 
hardware_interface::return_type AmrHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Convert commanded wheel angular velocity (rad/s) to the linear wheel-surface
  // velocity (cm/s) the firmware's CMD_SET_VELOCITY expects (see ControlMotor::setVelocity,
  // which compares this directly against the encoder-derived cm/s velocity).
  double radius_cm = wheel_radius_ * 100.0;
  float left_cm_s = static_cast<float>(wheel_velocity_commands_[0] * radius_cm);
  float right_cm_s = static_cast<float>(wheel_velocity_commands_[1] * radius_cm);
 
  // Skip re-sending an identical command every single cycle (e.g. holding still) --
  // still send at least once so the firmware's watchdog keeps seeing traffic
  // is not actually required here since read() already sends every cycle.
  if (left_cm_s == last_sent_commands_[0] && right_cm_s == last_sent_commands_[1]) {
    return hardware_interface::return_type::OK;
  }
 
  uint8_t data[8];
  std::memcpy(data + 0, &left_cm_s, 4);
  std::memcpy(data + 4, &right_cm_s, 4);
  if (sendPacket(protocol::CMD_SET_VELOCITY, data, sizeof(data))) {
    last_sent_commands_[0] = left_cm_s;
    last_sent_commands_[1] = right_cm_s;
  }
 
  return hardware_interface::return_type::OK;
}
 
}  // namespace uet_amr_hardware
 
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  uet_amr_hardware::AmrHardwareInterface,
  hardware_interface::SystemInterface)