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
#include "uet_amr_msgs/msg/hardware_status.hpp"

namespace uet_amr_hardware
{

/**
 * @brief Hardware interface for UET AMR differential drive robot.
 *
 * Communicates with the ESP32 base controller firmware via serial (UART)
 * using the push-style binary protocol in ros_protocol.h. The firmware
 * streams a feedback packet (measured wheel speed, raw per-wheel encoder
 * ticks, battery/temperature) every time it gets a fresh reading from the
 * hoverboard; there is no request/response. Per-wheel joint position is
 * built up here from the raw encoder ticks (see read()), and vehicle
 * odometry (x, y, theta) is left to diff_drive_controller rather than
 * trusting an onboard pose estimate.
 *
 * Topics exposed (via hardware interface):
 *   - /joint_states (via ros2_control)
 *   - /odom (via diff_drive_controller)
 *   - /hardware/status (uet_amr_msgs/msg/HardwareStatus, published directly by
 *     this node at 1 Hz from the latest decoded feedback packet)
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

    // Passively waits (up to timeout) for one valid feedback packet, used to
    // confirm the link is alive on configure/activate. There is no request to
    // send -- the firmware only speaks when it has something to say.
    bool waitForFeedback(std::chrono::milliseconds timeout);

    // Applies one decoded feedback packet to wheel_positions_ and the
    // per-cycle angle accumulators. Velocity is derived once per read() cycle
    // from the accumulators, since several packets can arrive in one cycle.
    void applyFeedback(const protocol::FeedbackPacket & pkt);

    // Publishes the latest feedback packet plus wheel state as HardwareStatus.
    // No battery voltage: the wire protocol only forwards a computed
    // percentage, not the hoverboard mainboard's raw batVoltage (see
    // protocol.hpp).
    void publishHardwareStatus();

    // Signed tick delta accounting for wrap-around at encoder_max_, matching
    // the firmware's own Odom::wrapDelta().
    int wrapTickDelta(int current, int previous) const;

    // Serial communication
    std::string serial_port_;
    int baud_rate_;
    int serial_fd_{-1};
    protocol::FeedbackParser feedback_parser_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::chrono::steady_clock::time_point last_feedback_time_;
    static constexpr std::chrono::milliseconds kFeedbackTimeout{500};

    // Wrap-aware encoder tick tracking (see wrapTickDelta()/applyFeedback()).
    bool have_last_ticks_{false};
    int last_tick_l_{0};
    int last_tick_r_{0};

    // Wheel angle accumulated since the last velocity update (see read()).
    double accum_rad_l_{0.0};
    double accum_rad_r_{0.0};
    std::chrono::steady_clock::time_point last_velocity_update_;

    // Status publishing. Owns a bare node (never spun -- publish() needs no
    // executor) since SystemInterface has no node of its own.
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<uet_amr_msgs::msg::HardwareStatus>::SharedPtr status_pub_;
    protocol::FeedbackPacket last_packet_{};
    bool have_packet_{false};
    std::chrono::steady_clock::time_point last_status_publish_;
    static constexpr std::chrono::seconds kStatusPeriod{1};
    static constexpr int kBatteryCriticalPercent = 15;
    static constexpr int kBatteryLowPercent = 30;

    // Wheel joint states
    std::vector<double> wheel_positions_;
    std::vector<double> wheel_velocities_;

    // Wheel commands
    std::vector<double> wheel_velocity_commands_;

    // Parameters
    // Raw hoverboard speed units per rad/s of wheel rotation. The hoverboard
    // responds differently to its common-mode (straight) and differential
    // (turning) channels, so each has its own measured gain.
    double motor_command_scale_linear_;
    double motor_command_scale_angular_;
    int ticks_per_rev_;          // encoder ticks per wheel revolution; must match firmware's Odom
    int encoder_max_;            // encoder wrap modulus; must match firmware's Odom
};

}  // namespace uet_amr_hardware
