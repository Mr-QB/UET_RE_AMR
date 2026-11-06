// Copyright (c) 2026 Department of Robotics, University of Engineering and
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
#include "uet_amr_msgs/msg/hardware_status.hpp"

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
#ifdef B460800
          case 460800: return B460800;
          case 921600: return B921600;
#else
          case 460800: return 460800;
          case 921600: return 921600;
#endif
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
    motor_command_scale_linear_ =
        std::stod(info_.hardware_parameters.at("motor_command_scale_linear"));
    motor_command_scale_angular_ =
        std::stod(info_.hardware_parameters.at("motor_command_scale_angular"));
    ticks_per_rev_ = std::stoi(info_.hardware_parameters.at("ticks_per_rev"));
    encoder_max_ = std::stoi(info_.hardware_parameters.at("encoder_max"));
    if (info_.hardware_parameters.count("motor_command_limit")) {
        motor_command_limit_ = std::stoi(info_.hardware_parameters.at("motor_command_limit"));
    }

    // Initialize state and command vectors (left, right)
    wheel_positions_.assign(2, 0.0);
    wheel_velocities_.assign(2, 0.0);
    wheel_velocity_commands_.assign(2, 0.0);

    // Bare node for status publishing only -- never spun, since publish()
    // needs no executor and this node has no subscriptions/services.
    node_ = rclcpp::Node::make_shared("amr_hardware_interface_status");
    status_pub_ = node_->create_publisher<uet_amr_msgs::msg::HardwareStatus>(
        "/hardware/status", 10);

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

    // The base MCU may be powered after this node starts (or be unplugged and
    // replugged later). Never fail configure over it: read() keeps retrying.
    if (!openSerial() || !waitForFeedback(1000ms)) {
        RCLCPP_WARN(
            logger(), "Firmware not available on '%s' yet; will keep retrying in the background.",
            serial_port_.c_str());
        closeSerial();
        return hardware_interface::CallbackReturn::SUCCESS;
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
    if (serial_fd_ >= 0 && !waitForFeedback(500ms)) {
        RCLCPP_WARN(logger(), "Firmware not responding at activation; read() will keep retrying");
    }

    have_last_ticks_ = false; // don't compute a bogus delta against a stale sample
    last_feedback_time_ = std::chrono::steady_clock::now();
    last_velocity_update_ = last_feedback_time_;
    accum_rad_l_ = 0.0;
    accum_rad_r_ = 0.0;
    accum_dt_ = 0.0;
    have_stamp_ = false;
    last_reconnect_attempt_ = last_feedback_time_;
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

void AmrHardwareInterface::applyFeedback(const protocol::FeedbackPacket & pkt)
{
    last_packet_ = pkt;
    have_packet_ = true;

    if (have_stamp_) {
        const unsigned missed = (pkt.seq - last_seq_ - 1u) & 0x7Fu;
        if (missed != 0) {
            RCLCPP_WARN_THROTTLE(
                logger(), steady_clock_, 1000, "Lost %u feedback frame(s)", missed);
        }
        // 11-bit stamp: masked subtraction is wrap-safe as long as packets
        // arrive within the ~524 ms wrap (the feedback timeout is 500 ms).
        accum_dt_ += static_cast<double>((pkt.stamp - last_stamp_) & protocol::kStampMask) *
            protocol::kStampUnitSeconds;
    }
    last_stamp_ = pkt.stamp;
    last_seq_ = pkt.seq;
    have_stamp_ = true;

    if (!have_last_ticks_) {
        last_tick_l_ = pkt.en_tick_l;
        last_tick_r_ = pkt.en_tick_r;
        have_last_ticks_ = true;
        accum_dt_ = 0.0;
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

    accum_rad_l_ += new_left_pos - wheel_positions_[0];
    accum_rad_r_ += new_right_pos - wheel_positions_[1];
    wheel_positions_[0] = new_left_pos;
    wheel_positions_[1] = new_right_pos;
}

void AmrHardwareInterface::publishHardwareStatus()
{
    const auto & pkt = last_packet_;
    uet_amr_msgs::msg::HardwareStatus msg;
    msg.header.stamp = node_->now();
    const int battery_percent = static_cast<int>(pkt.battery);
    msg.battery_percentage = static_cast<float>(battery_percent) / 100.0f;
    msg.estop = pkt.estop != 0;
    msg.current = static_cast<float>(pkt.current_a) / 100.0f;
    msg.temperature = static_cast<float>(pkt.temp_c);
    msg.left_speed = static_cast<int16_t>(pkt.speed_l);
    msg.right_speed = static_cast<int16_t>(pkt.speed_r);
    msg.left_wheel_position = wheel_positions_[0];
    msg.right_wheel_position = wheel_positions_[1];
    msg.left_wheel_velocity = wheel_velocities_[0];
    msg.right_wheel_velocity = wheel_velocities_[1];

    if (battery_percent <= kBatteryCriticalPercent) {
        msg.battery_state = "CRITICAL";
    } else if (battery_percent <= kBatteryLowPercent) {
        msg.battery_state = "LOW";
    } else {
        msg.battery_state = "OK";
    }

    status_pub_->publish(msg);
}

hardware_interface::return_type AmrHardwareInterface::reconnect()
{
    wheel_velocities_.assign(2, 0.0);
    const auto now = std::chrono::steady_clock::now();
    if (now - last_reconnect_attempt_ < kReconnectPeriod) {
        return hardware_interface::return_type::OK;
    }
    last_reconnect_attempt_ = now;

    closeSerial();
    if (!openSerial()) {
        return hardware_interface::return_type::OK;
    }
    RCLCPP_INFO(logger(), "Reopened '%s'; waiting for firmware.", serial_port_.c_str());
    feedback_parser_ = protocol::FeedbackParser();
    have_last_ticks_ = false;  // no bogus tick delta across the gap
    have_stamp_ = false;
    last_feedback_time_ = now;  // restart the feedback timeout
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmrHardwareInterface::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    if (serial_fd_ < 0) {
        return reconnect();
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
            closeSerial();  // device vanished (USB unplug); reopen from read()
            return reconnect();
        }
        if (n == 0) {
            break;
        }
        for (ssize_t i = 0; i < n; ++i) {
            if (feedback_parser_.feed(buf[i])) {
                applyFeedback(feedback_parser_.packet());
                got_packet = true;
            }
        }
    }

    const auto now = std::chrono::steady_clock::now();
    if (got_packet) {
        last_feedback_time_ = now;
        // Differencing 90-tick/rev encoder counts over one packet quantizes the speed into large
        // steps (a single tick over a few ms reads several rad/s), which made the drive jerky.
        // The hoverboard's own measured rpm is far finer, so use it for velocity.
        constexpr double kRpmToRadPerSec = 2.0 * M_PI / 60.0;
        wheel_velocities_[0] = last_packet_.speed_l * kRpmToRadPerSec;
        wheel_velocities_[1] = last_packet_.speed_r * kRpmToRadPerSec;
        accum_rad_l_ = 0.0;
        accum_rad_r_ = 0.0;
        accum_dt_ = 0.0;
    }

    if (have_packet_ && now - last_status_publish_ >= kStatusPeriod) {
        publishHardwareStatus();
        last_status_publish_ = now;
    }

    if (got_packet) {
        return hardware_interface::return_type::OK;
    }

    const auto since_last =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_time_);
    if (since_last > kFeedbackTimeout) {
        RCLCPP_ERROR_THROTTLE(
            logger(), steady_clock_, 1000,
            "No feedback from firmware for %ldms", static_cast<long>(since_last.count()));
        closeSerial();  // reopen from read() in case the port went stale
        return reconnect();
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmrHardwareInterface::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    // The hoverboard mixes (L+R)/2 as speed and (R-L)/2 as steer with
    // different effective gains, so scale the two components separately.
    const double common = 0.5 * (wheel_velocity_commands_[0] + wheel_velocity_commands_[1]);
    const double diff = 0.5 * (wheel_velocity_commands_[1] - wheel_velocity_commands_[0]);
    const double left_raw = common * motor_command_scale_linear_ - diff * motor_command_scale_angular_;
    const double right_raw = common * motor_command_scale_linear_ + diff * motor_command_scale_angular_;

    const double limit = static_cast<double>(motor_command_limit_);
    const int16_t left_speed = static_cast<int16_t>(std::clamp(left_raw, -limit, limit));
    const int16_t right_speed = static_cast<int16_t>(std::clamp(right_raw, -limit, limit));
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
