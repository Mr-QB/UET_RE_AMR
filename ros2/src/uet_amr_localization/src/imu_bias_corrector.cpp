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

// Removes the gyro zero-rate bias from an IMU stream before the EKF integrates it.
//
// The D435i gyro reads ~0.002 rad/s on yaw at rest, which the EKF integrates into ~7 deg/min of heading drift while the
// robot sits still. This node estimates the per-axis bias with a slow low-pass filter, but only while the robot is known
// to be stationary: wheel odometry speed ~0, no commanded motion, and the gyro itself quiet. The bias is frozen whenever
// the robot moves (or the wheels spin: odometry then reports motion), so rotation is never learned as bias.
//
// C++ port of the earlier imu_bias_corrector.py (same parameters and behaviour); at 200 Hz the Python node used about a
// third of a core on message handling alone.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuBiasCorrector : public rclcpp::Node
{
public:
  ImuBiasCorrector()
  : Node("imu_bias_corrector")
  {
    const auto imu_in = declare_parameter<std::string>("imu_in", "camera/camera/imu");
    const auto imu_out = declare_parameter<std::string>("imu_out", "imu/corrected");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "diff_drive_controller/odom");
    still_lin_ = declare_parameter<double>("still_lin_mps", 0.003);    // wheel odom speeds below this are stationary
    still_ang_ = declare_parameter<double>("still_ang_rps", 0.004);
    hold_ = declare_parameter<double>("still_hold_s", 1.0);            // must stay still this long before learning
    quiet_ = declare_parameter<double>("gyro_quiet_rps", 0.02);        // reject samples where the gyro says it moves
    tau_ = declare_parameter<double>("tau_s", 8.0);                    // bias low-pass time constant
    seed_s_ = declare_parameter<double>("initial_bias_s", 2.0);        // first this-much still time seeds the bias

    pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_out, rclcpp::SensorDataQoS());
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_in, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
    RCLCPP_INFO(get_logger(), "IMU bias corrector: %s -> %s", imu_in.c_str(), imu_out.c_str());
  }

private:
  double nowSec()
  {
    return get_clock()->now().nanoseconds() * 1e-9;
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const double v = std::abs(msg.twist.twist.linear.x);
    const double w = std::abs(msg.twist.twist.angular.z);
    if (v < still_lin_ && w < still_ang_) {
      if (!still_since_) {
        still_since_ = nowSec();
      }
      moving_ = false;
    } else {
      still_since_.reset();
      moving_ = true;
    }
  }

  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    const double now = nowSec();
    const double dt = last_t_ ? std::min(now - *last_t_, 0.1) : 0.0;
    last_t_ = now;
    const std::array<double, 3> g{msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};

    bool quiet = true;
    for (size_t i = 0; i < 3; ++i) {
      quiet = quiet && std::abs(g[i] - bias_[i]) < quiet_;
    }
    const bool still = !moving_ && still_since_ && now - *still_since_ >= hold_ && quiet;
    if (still && dt > 0.0) {
      double a;
      if (seeded_time_ < seed_s_) {   // fast start: running mean over the first seconds of stillness
        seeded_time_ += dt;
        a = dt / std::max(seeded_time_, dt);
      } else {
        a = dt / tau_;
      }
      for (size_t i = 0; i < 3; ++i) {
        bias_[i] += a * (g[i] - bias_[i]);
      }
      if (now - last_log_ > 20.0) {
        last_log_ = now;
        RCLCPP_INFO(
          get_logger(), "gyro bias (rad/s): x=%.5f y=%.5f z=%.5f (%.3f deg/s yaw)",
          bias_[0], bias_[1], bias_[2], bias_[2] * 180.0 / M_PI);
      }
    }

    auto out = std::make_unique<sensor_msgs::msg::Imu>(msg);
    out->angular_velocity.x = g[0] - bias_[0];
    out->angular_velocity.y = g[1] - bias_[1];
    out->angular_velocity.z = g[2] - bias_[2];
    pub_->publish(std::move(out));
  }

  double still_lin_, still_ang_, hold_, quiet_, tau_, seed_s_;
  std::array<double, 3> bias_{0.0, 0.0, 0.0};
  std::optional<double> still_since_;
  std::optional<double> last_t_;
  double seeded_time_{0.0};
  double last_log_{0.0};
  bool moving_{true};   // assume moving until odometry says otherwise

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuBiasCorrector>());
  rclcpp::shutdown();
  return 0;
}
