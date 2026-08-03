// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#include "uet_amr_hardware/amr_hardware_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace uet_amr_hardware
{

using namespace std::chrono_literals;

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("AmrHardwareInterface");
}
}  // namespace

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

  RCLCPP_INFO(logger(),
    "Initialized: port=%s, baud=%d", serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool AmrHardwareInterface::openSerial()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      logger(), "Failed to open serial port '%s': %s",
      serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  if (baud_rate_ != 115200) {
    RCLCPP_WARN(
      logger(),
      "Requested baud_rate=%d but firmware protocol is fixed at 115200; using 115200 anyway.",
      baud_rate_);
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(logger(), "tcgetattr failed on '%s': %s", serial_port_.c_str(),
      std::strerror(errno));
    closeSerial();
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger(), "tcsetattr failed on '%s': %s", serial_port_.c_str(),
      std::strerror(errno));
    closeSerial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void AmrHardwareInterface::closeSerial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool AmrHardwareInterface::sendFrame(protocol::Cmd cmd, const uint8_t * data, uint8_t len)
{
  if (serial_fd_ < 0) {
    return false;
  }
  std::vector<uint8_t> frame = protocol::encodeFrame(cmd, data, len);

  size_t written = 0;
  while (written < frame.size()) {
    ssize_t n = ::write(serial_fd_, frame.data() + written, frame.size() - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR_THROTTLE(
        logger(), steady_clock_, 1000,
        "write() to '%s' failed: %s", serial_port_.c_str(), std::strerror(errno));
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

bool AmrHardwareInterface::requestAndWait(
  protocol::Cmd request_cmd, const uint8_t * payload, uint8_t payload_len,
  protocol::Cmd expect_reply_cmd, std::vector<uint8_t> & out_data,
  std::chrono::milliseconds timeout)
{
  if (!sendFrame(request_cmd, payload, payload_len)) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);

    pollfd pfd{};
    pfd.fd = serial_fd_;
    pfd.events = POLLIN;
    int ret = ::poll(&pfd, 1, static_cast<int>(remaining.count()));
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (ret == 0) {
      return false;  // timed out
    }

    uint8_t buf[64];
    ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }

    for (ssize_t i = 0; i < n; ++i) {
      if (parser_.feed(buf[i])) {
        if (parser_.cmd() == expect_reply_cmd) {
          out_data.assign(parser_.data(), parser_.data() + parser_.len());
          return true;
        }
        // Some other valid frame (stray ACK, unsolicited watchdog push, ...) - discard.
      }
    }
  }
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger(), "Configuring hardware interface...");

  if (!openSerial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool ok = false;
  std::vector<uint8_t> reply;
  for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
    ok = requestAndWait(protocol::Cmd::Ping, nullptr, 0, protocol::Cmd::AckPing, reply, 100ms);
  }
  if (!ok) {
    RCLCPP_ERROR(
      logger(), "No response to CMD_PING from '%s' after 5 attempts", serial_port_.c_str());
    closeSerial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  consecutive_comm_failures_ = 0;
  RCLCPP_INFO(logger(), "Firmware responded to ping on '%s'.", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bool ok = false;
  std::vector<uint8_t> reply;
  for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
    ok = requestAndWait(protocol::Cmd::Start, nullptr, 0, protocol::Cmd::AckControl, reply, 50ms);
  }
  if (!ok) {
    RCLCPP_ERROR(logger(), "Firmware did not ACK CMD_START");
    return hardware_interface::CallbackReturn::ERROR;
  }

  consecutive_comm_failures_ = 0;
  RCLCPP_INFO(logger(), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<uint8_t> reply;
  bool ok = requestAndWait(protocol::Cmd::Stop, nullptr, 0, protocol::Cmd::AckControl, reply,
      100ms);
  if (!ok) {
    RCLCPP_WARN(
      logger(), "No ACK for CMD_STOP -- firmware watchdog will stop it within 200ms regardless");
  }

  RCLCPP_INFO(logger(), "Hardware deactivated.");
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
  std::vector<uint8_t> reply;
  bool ok = requestAndWait(
    protocol::Cmd::EncoderPos, nullptr, 0, protocol::Cmd::EncoderPos, reply, 15ms);

  if (!ok || reply.size() < 8) {
    consecutive_comm_failures_++;
    RCLCPP_WARN_THROTTLE(
      logger(), steady_clock_, 1000,
      "No CMD_ENCODER_POS reply from firmware (failure #%d)", consecutive_comm_failures_);
    if (consecutive_comm_failures_ > kMaxConsecutiveFailures) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  consecutive_comm_failures_ = 0;

  int32_t left_ticks = 0;
  int32_t right_ticks = 0;
  std::memcpy(&left_ticks, reply.data() + 0, 4);
  std::memcpy(&right_ticks, reply.data() + 4, 4);

  const double ticks_to_rad = 2.0 * M_PI / encoder_ticks_per_rev_;
  const double new_left_pos = left_ticks * ticks_to_rad;
  const double new_right_pos = right_ticks * ticks_to_rad;

  const double dt = period.seconds();
  if (dt > 1e-6) {
    wheel_velocities_[0] = (new_left_pos - wheel_positions_[0]) / dt;
    wheel_velocities_[1] = (new_right_pos - wheel_positions_[1]) / dt;
  }
  wheel_positions_[0] = new_left_pos;
  wheel_positions_[1] = new_right_pos;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmrHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  const float left_cm_s = static_cast<float>(wheel_velocity_commands_[0] * wheel_radius_ * 100.0);
  const float right_cm_s =
    static_cast<float>(wheel_velocity_commands_[1] * wheel_radius_ * 100.0);

  uint8_t payload[8];
  std::memcpy(payload + 0, &left_cm_s, 4);
  std::memcpy(payload + 4, &right_cm_s, 4);

  // Fire-and-forget every cycle (even zero velocity) to keep firmware's
  // 200ms watchdog fed. Any reply/ACK sitting in the RX buffer is picked
  // up and discarded by the next read() cycle's requestAndWait().
  sendFrame(protocol::Cmd::SetVelocity, payload, sizeof(payload));

  return hardware_interface::return_type::OK;
}

}  // namespace uet_amr_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  uet_amr_hardware::AmrHardwareInterface,
  hardware_interface::SystemInterface)
