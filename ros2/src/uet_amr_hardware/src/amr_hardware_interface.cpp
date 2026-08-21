// Copyright (c) 2026 UET Robotics Club, University of Engineering and
//                    Technology, Vietnam National University, Hanoi (VNU).
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

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
#include "uet_amr_msgs/msg/battery_status.hpp"

namespace uet_amr_hardware
{

using namespace std::chrono_literals;

namespace
{
    rclcpp::Logger logger()
    {
        return rclcpp::get_logger("AmrHardwareInterface");
    }

    speed_t baudRateToSpeed(int baud_rate)
    {
        switch (baud_rate) {
          case 9600: return B9600;
          case 19200: return B19200;
          case 38400: return B38400;
          case 57600: return B57600;
          case 115200: return B115200;
          case 230400: return B230400;
          case 460800: return B460800;
          case 921600: return B921600;
          default: return B0;
        }
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
    serial_port_ = info_.hardware_parameters.at("serial_port");
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    motor_command_scale_ = std::stod(info_.hardware_parameters.at("motor_command_scale"));
    ticks_per_rev_ = std::stoi(info_.hardware_parameters.at("ticks_per_rev"));
    encoder_max_ = std::stoi(info_.hardware_parameters.at("encoder_max"));

    // Initialize state and command vectors (left, right)
    wheel_positions_.assign(2, 0.0);
    wheel_velocities_.assign(2, 0.0);
    wheel_velocity_commands_.assign(2, 0.0);

    // Bare node for battery status publishing only -- never spun, since
    // publish() needs no executor and this node has no subscriptions/services.
    node_ = rclcpp::Node::make_shared("amr_hardware_interface_battery");
    battery_pub_ = node_->create_publisher<uet_amr_msgs::msg::BatteryStatus>(
        "/battery/status", 10);

    RCLCPP_INFO(
        logger(),
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

    const speed_t speed = baudRateToSpeed(baud_rate_);
    if (speed == B0) {
        RCLCPP_ERROR(
            logger(),
            "Unsupported baud_rate=%d on '%s'; supported rates: 9600, 19200, 38400, 57600, "
            "115200, 230400, 460800, 921600.",
            baud_rate_, serial_port_.c_str());
        closeSerial();
        return false;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(
            logger(), "tcgetattr failed on '%s': %s", serial_port_.c_str(),
            std::strerror(errno));
        closeSerial();
        return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(
            logger(), "tcsetattr failed on '%s': %s", serial_port_.c_str(),
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

bool AmrHardwareInterface::sendCommand(int16_t speed_left, int16_t speed_right)
{
    if (serial_fd_ < 0) {
        return false;
    }
    protocol::CommandFrame frame = protocol::encodeCommandFrame(speed_left, speed_right);

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

bool AmrHardwareInterface::waitForFeedback(std::chrono::milliseconds timeout)
{
    protocol::FeedbackParser probe_parser;
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
            return false; // timed out
        }

        uint8_t buf[64];
        ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
        if (n <= 0) {
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            if (probe_parser.feed(buf[i])) {
                return true;
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

    if (!waitForFeedback(1000ms)) {
        RCLCPP_ERROR(
            logger(), "No feedback from firmware on '%s' within 1s", serial_port_.c_str());
        closeSerial();
        return hardware_interface::CallbackReturn::ERROR;
    }

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
    if (!waitForFeedback(500ms)) {
        RCLCPP_ERROR(logger(), "Firmware not responding, aborting activation");
        return hardware_interface::CallbackReturn::ERROR;
    }

    have_last_ticks_ = false; // don't compute a bogus delta against a stale sample
    last_feedback_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(logger(), "Hardware activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Firmware has no separate stop/ack handshake -- commanding zero velocity
    // is what actually stops the wheels. Best-effort: even if this send fails,
    // the firmware's own no-traffic watchdog will stop it shortly after.
    sendCommand(0, 0);

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

int AmrHardwareInterface::wrapTickDelta(int current, int previous) const
{
    int delta = current - previous;
    const int half = encoder_max_ / 2;
    if (delta > half) {
        delta -= encoder_max_;
    } else if (delta < -half) {
        delta += encoder_max_;
    }
    return delta;
}

void AmrHardwareInterface::applyFeedback(
    const protocol::FeedbackPacket & pkt, const rclcpp::Duration & period)
{
    publishBatteryStatus(pkt);

    if (!have_last_ticks_) {
        last_tick_l_ = pkt.en_tick_l;
        last_tick_r_ = pkt.en_tick_r;
        have_last_ticks_ = true;
        return;
    }

    const int delta_l = -wrapTickDelta(pkt.en_tick_l, last_tick_l_);
    const int delta_r = -wrapTickDelta(pkt.en_tick_r, last_tick_r_);
    last_tick_l_ = pkt.en_tick_l;
    last_tick_r_ = pkt.en_tick_r;

    // Ticks -> wheel angle, matching the firmware's own Odom::dist_per_tick_
    // (2*pi*wheel_radius/ticks_per_rev) with the radius factored out since
    // wheel_positions_ is an angle, not a distance.
    const double rad_per_tick = 2.0 * M_PI / static_cast<double>(ticks_per_rev_);
    const double new_left_pos = wheel_positions_[0] + delta_l * rad_per_tick;
    const double new_right_pos = wheel_positions_[1] + delta_r * rad_per_tick;

    const double dt = period.seconds();
    if (dt > 1e-6) {
        wheel_velocities_[0] = (new_left_pos - wheel_positions_[0]) / dt;
        wheel_velocities_[1] = (new_right_pos - wheel_positions_[1]) / dt;
    }
    wheel_positions_[0] = new_left_pos;
    wheel_positions_[1] = new_right_pos;
}

void AmrHardwareInterface::publishBatteryStatus(const protocol::FeedbackPacket & pkt)
{
    uet_amr_msgs::msg::BatteryStatus msg;
    msg.header.stamp = node_->now();
    msg.current = static_cast<float>(pkt.current_a) / 100.0f;
    msg.percentage = static_cast<float>(pkt.battery) / 100.0f;
    msg.temperature = static_cast<float>(pkt.temp_c);
    msg.is_charging = pkt.charging != 0;

    if (static_cast<int>(pkt.battery) <= kBatteryCriticalPercent) {
        msg.status = "CRITICAL";
    } else if (static_cast<int>(pkt.battery) <= kBatteryLowPercent) {
        msg.status = "LOW";
    } else {
        msg.status = "OK";
    }

    battery_pub_->publish(msg);
}

hardware_interface::return_type AmrHardwareInterface::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
    if (serial_fd_ < 0) {
        return hardware_interface::return_type::ERROR;
    }

    bool got_packet = false;
    uint8_t buf[128];
    while (true) {
        ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break; // no more data available right now
            }
            if (errno == EINTR) {
                continue;
            }
            RCLCPP_ERROR_THROTTLE(
                logger(), steady_clock_, 1000,
                "read() from '%s' failed: %s", serial_port_.c_str(), std::strerror(errno));
            return hardware_interface::return_type::ERROR;
        }
        if (n == 0) {
            break;
        }
        for (ssize_t i = 0; i < n; ++i) {
            if (feedback_parser_.feed(buf[i])) {
                applyFeedback(feedback_parser_.packet(), period);
                got_packet = true;
            }
        }
    }

    const auto now = std::chrono::steady_clock::now();
    if (got_packet) {
        last_feedback_time_ = now;
        return hardware_interface::return_type::OK;
    }

    const auto since_last =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_time_);
    if (since_last > kFeedbackTimeout) {
        RCLCPP_ERROR_THROTTLE(
            logger(), steady_clock_, 1000,
            "No feedback from firmware for %ldms", static_cast<long>(since_last.count()));
        return hardware_interface::return_type::ERROR;
    }

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

    RCLCPP_INFO_THROTTLE(
        logger(), steady_clock_, 200,
        "cmd: left=%.3f right=%.3f -> raw left=%d right=%d",
        wheel_velocity_commands_[0], wheel_velocity_commands_[1],
        static_cast<int>(left_speed), static_cast<int>(right_speed));

    // Fire-and-forget every cycle (even zero velocity) -- firmware never
    // replies to a command frame, and this is what keeps its no-traffic
    // watchdog fed.
    sendCommand(left_speed, right_speed);

    return hardware_interface::return_type::OK;
}

}  // namespace uet_amr_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    uet_amr_hardware::AmrHardwareInterface,
    hardware_interface::SystemInterface)
