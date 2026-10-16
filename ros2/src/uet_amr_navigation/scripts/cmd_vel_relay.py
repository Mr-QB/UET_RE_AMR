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
Forward Nav2's recovery-behavior velocity commands to the drive controller.

nav2_bringup's behavior_server (spin, backup, drive_on_heading) publishes on /cmd_vel, while only the controller
output goes through velocity_smoother, which navigation.launch.py remaps onto the drive controller's topic. Nothing
consumed /cmd_vel, so every recovery "ran" without moving the robot and failed with a timeout. A global remap of
cmd_vel is not an option: it would also override the controller's and smoother's own cmd_vel -> cmd_vel_nav remaps.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self._pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('input_topic').value, self._pub.publish, 10)
        self.get_logger().info(
            f'{self.get_parameter("input_topic").value} -> {self.get_parameter("output_topic").value}')


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
