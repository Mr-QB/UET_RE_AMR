// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#include "uet_amr_hardware/amr_hardware_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
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

double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
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
  motor_command_scale_ = std::stod(info_.hardware_parameters.at("motor_command_scale"));

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

bool AmrHardwareInterface::sendCommand(protocol::Cmd cmd, int16_t speed_left, int16_t speed_right)
{
  if (serial_fd_ < 0) {
    return false;
  }
  protocol::CommandFrame frame = protocol::encodeCommandFrame(cmd, speed_left, speed_right);

  size_t written = 0;
  while (written < sizeof(frame.bytes)) {
    ssize_t n = ::write(serial_fd_, frame.bytes + written, sizeof(frame.bytes) - written);
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
  protocol::Cmd request_cmd, protocol::Cmd expect_reply_cmd,
  std::vector<uint8_t> & out_payload, std::chrono::milliseconds timeout)
{
  if (!sendCommand(request_cmd, 0, 0)) {
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
          out_payload.assign(parser_.payload(), parser_.payload() + parser_.payloadLen());
          return true;
        }
        // Some other valid frame -- discard and keep waiting.
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
    ok = requestAndWait(
      protocol::Cmd::RequestStatus, protocol::Cmd::RequestStatus, reply, 100ms);
  }
  if (!ok) {
    RCLCPP_ERROR(
      logger(), "No response from firmware on '%s' after 5 attempts", serial_port_.c_str());
    closeSerial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  consecutive_comm_failures_ = 0;
  RCLCPP_INFO(logger(), "Firmware responded on '%s'.", serial_port_.c_str());
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
    ok = requestAndWait(
      protocol::Cmd::RequestStatus, protocol::Cmd::RequestStatus, reply, 50ms);
  }
  if (!ok) {
    RCLCPP_ERROR(logger(), "Firmware not responding, aborting activation");
    return hardware_interface::CallbackReturn::ERROR;
  }

  consecutive_comm_failures_ = 0;
  have_last_pose_ = false;  // don't compute a bogus delta against a stale sample
  RCLCPP_INFO(logger(), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Firmware has no separate stop/ack handshake -- commanding zero velocity
  // is what actually stops the wheels. Best-effort: even if this send fails,
  // the firmware's own 1000ms no-traffic watchdog will stop it shortly after.
  sendCommand(protocol::Cmd::SetVelocity, 0, 0);

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
    protocol::Cmd::RequestOdometry, protocol::Cmd::RequestOdometry, reply, 15ms);

  if (!ok || reply.size() < 10) {
    consecutive_comm_failures_++;
    RCLCPP_WARN_THROTTLE(
      logger(), steady_clock_, 1000,
      "No odometry reply from firmware (failure #%d)", consecutive_comm_failures_);
    if (consecutive_comm_failures_ > kMaxConsecutiveFailures) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  consecutive_comm_failures_ = 0;

  float x_cm = 0.0f;
  float y_cm = 0.0f;
  int16_t theta_raw = 0;
  std::memcpy(&x_cm, reply.data() + 0, 4);
  std::memcpy(&y_cm, reply.data() + 4, 4);
  std::memcpy(&theta_raw, reply.data() + 8, 2);
  const double theta_rad = static_cast<double>(theta_raw) / 10.0;

  if (!have_last_pose_) {
    last_x_cm_ = x_cm;
    last_y_cm_ = y_cm;
    last_theta_rad_ = theta_rad;
    have_last_pose_ = true;
    return hardware_interface::return_type::OK;
  }

  // Recover per-wheel displacement by inverting the same diff-drive
  // kinematics the firmware used to build (x, y, theta) from its encoder
  // deltas in the first place (see Odom::integrate() in odom.cpp).
  const double wheel_radius_cm = wheel_radius_ * 100.0;
  const double wheel_separation_cm = wheel_separation_ * 100.0;

  const double dtheta = normalizeAngle(theta_rad - last_theta_rad_);
  const double theta_mid = last_theta_rad_ + dtheta / 2.0;
  const double dc_cm =
    (x_cm - last_x_cm_) * std::cos(theta_mid) + (y_cm - last_y_cm_) * std::sin(theta_mid);
  const double dl_cm = dc_cm - dtheta * wheel_separation_cm / 2.0;
  const double dr_cm = dc_cm + dtheta * wheel_separation_cm / 2.0;

  const double new_left_pos = wheel_positions_[0] + dl_cm / wheel_radius_cm;
  const double new_right_pos = wheel_positions_[1] + dr_cm / wheel_radius_cm;

  const double dt = period.seconds();
  if (dt > 1e-6) {
    wheel_velocities_[0] = (new_left_pos - wheel_positions_[0]) / dt;
    wheel_velocities_[1] = (new_right_pos - wheel_positions_[1]) / dt;
  }
  wheel_positions_[0] = new_left_pos;
  wheel_positions_[1] = new_right_pos;

  last_x_cm_ = x_cm;
  last_y_cm_ = y_cm;
  last_theta_rad_ = theta_rad;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmrHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  const double left_raw = wheel_velocity_commands_[0] * motor_command_scale_;
  const double right_raw = wheel_velocity_commands_[1] * motor_command_scale_;

  const int16_t left_speed = static_cast<int16_t>(
    std::clamp(left_raw, -32768.0, 32767.0));
  const int16_t right_speed = static_cast<int16_t>(
    std::clamp(right_raw, -32768.0, 32767.0));

  // Fire-and-forget every cycle (even zero velocity) -- firmware never
  // replies to SetVelocity, and this is what keeps its 1000ms no-traffic
  // watchdog fed.
  sendCommand(protocol::Cmd::SetVelocity, left_speed, right_speed);

  return hardware_interface::return_type::OK;
}

}  // namespace uet_amr_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  uet_amr_hardware::AmrHardwareInterface,
  hardware_interface::SystemInterface)
