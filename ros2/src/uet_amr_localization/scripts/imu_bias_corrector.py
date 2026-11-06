#!/usr/bin/env python3
# Copyright (c) 2026 Department of Robotics, University of Engineering and
#                    Technology, Vietnam National University, Hanoi (VNU).
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

"""
Removes the gyro zero-rate bias from an IMU stream before the EKF integrates it.

The D435i gyro reads ~0.002 rad/s on yaw at rest, which the EKF integrates into ~7 deg/min of heading drift while the robot
sits still. This node estimates the per-axis bias with a slow low-pass filter, but only while the robot is known to be
stationary: wheel odometry speed ~0, no commanded motion, and the gyro itself quiet. The bias is frozen whenever the robot moves
(or the wheels spin: odometry then reports motion), so rotation is never learned as bias.
"""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ImuBiasCorrector(Node):
    def __init__(self):
        super().__init__('imu_bias_corrector')
        self.declare_parameter('imu_in', 'camera/camera/imu')
        self.declare_parameter('imu_out', 'imu/corrected')
        self.declare_parameter('odom_topic', 'diff_drive_controller/odom')
        self.declare_parameter('still_lin_mps', 0.003)   # wheel odom speeds below this count as stationary
        self.declare_parameter('still_ang_rps', 0.004)
        self.declare_parameter('still_hold_s', 1.0)      # must stay still this long before learning
        self.declare_parameter('gyro_quiet_rps', 0.02)   # reject samples where the gyro itself says it is moving
        self.declare_parameter('tau_s', 8.0)             # bias low-pass time constant
        self.declare_parameter('initial_bias_s', 2.0)    # first this-much still time seeds the bias directly

        p = self.get_parameter
        self.still_lin = p('still_lin_mps').value
        self.still_ang = p('still_ang_rps').value
        self.hold = p('still_hold_s').value
        self.quiet = p('gyro_quiet_rps').value
        self.tau = p('tau_s').value
        self.seed_s = p('initial_bias_s').value

        self.bias = [0.0, 0.0, 0.0]
        self.still_since = None
        self.seeded_time = 0.0
        self.last_t = None
        self.last_log = 0.0

        self.pub = self.create_publisher(Imu, p('imu_out').value, qos_profile_sensor_data)
        self.create_subscription(Imu, p('imu_in').value, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic').value, self.on_odom, 10)
        self.moving = True   # assume moving until odometry says otherwise
        self.get_logger().info('IMU bias corrector: %s -> %s' % (p('imu_in').value, p('imu_out').value))

    def on_odom(self, msg):
        v = abs(msg.twist.twist.linear.x)
        w = abs(msg.twist.twist.angular.z)
        now = self.get_clock().now().nanoseconds * 1e-9
        if v < self.still_lin and w < self.still_ang:
            if self.still_since is None:
                self.still_since = now
            self.moving = False
        else:
            self.still_since = None
            self.moving = True

    def on_imu(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_t is None else min(now - self.last_t, 0.1)
        self.last_t = now
        g = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

        still = (not self.moving and self.still_since is not None and now - self.still_since >= self.hold
                 and max(abs(g[i] - self.bias[i]) for i in range(3)) < self.quiet)
        if still and dt > 0.0:
            if self.seeded_time < self.seed_s:   # fast start: running mean over the first seconds of stillness
                self.seeded_time += dt
                a = dt / max(self.seeded_time, dt)
            else:
                a = dt / self.tau
            for i in range(3):
                self.bias[i] += a * (g[i] - self.bias[i])
            if now - self.last_log > 20.0:
                self.last_log = now
                self.get_logger().info('gyro bias (rad/s): x=%.5f y=%.5f z=%.5f (%.3f deg/s yaw)' %
                                       (*self.bias, math.degrees(self.bias[2])))

        out = copy.copy(msg)
        out.angular_velocity.x = g[0] - self.bias[0]
        out.angular_velocity.y = g[1] - self.bias[1]
        out.angular_velocity.z = g[2] - self.bias[2]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuBiasCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
