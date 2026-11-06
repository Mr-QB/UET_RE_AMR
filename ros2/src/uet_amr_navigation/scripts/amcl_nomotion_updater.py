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

"""
Keep AMCL localizing while the robot is not moving.

AMCL only updates after the robot has moved update_min_d / turned update_min_a. When the robot stalls (here: climbing the door
ramp, wheels slow or slipping) wheel odometry keeps drifting, the scan stops matching the map, and because the robot is not
moving AMCL never corrects it: the lidar marks land in the wrong place in the global costmap, doorways look blocked and the
robot cannot recover. This node calls AMCL's request_nomotion_update service at update_rate_hz while the robot is slower than
stationary_speed, so the particle filter keeps using the scan. (Measured: with the robot standing still, scan points within
10 cm of a map wall went from 27 % to 59 % in 25 s.) While the robot moves AMCL updates by itself.

Parameters
  odom_topic          (default /odometry/filtered)
  service             (default /request_nomotion_update)
  update_rate_hz      (default 2.0)
  stationary_speed    m/s and rad/s below which the robot counts as not moving (default 0.05)
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Empty


class AmclNomotionUpdater(Node):
    def __init__(self):
        super().__init__('amcl_nomotion_updater')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('service', '/request_nomotion_update')
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('stationary_speed', 0.05)
        p = self.get_parameter
        self.limit = float(p('stationary_speed').value)
        self.speed = 0.0
        self.pending = None
        self.client = self.create_client(Empty, p('service').value)
        self.create_subscription(Odometry, p('odom_topic').value, self._on_odom, 10)
        self.create_timer(1.0 / float(p('update_rate_hz').value), self._tick)
        self.get_logger().info(f'forcing AMCL updates at {p("update_rate_hz").value} Hz while slower than {self.limit} m/s')

    def _on_odom(self, m):
        self.speed = max(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), abs(m.twist.twist.angular.z))

    def _tick(self):
        if self.speed > self.limit or not self.client.service_is_ready():
            return
        if self.pending is not None and not self.pending.done():
            return    # the last request is still being processed
        self.pending = self.client.call_async(Empty.Request())


def main():
    rclpy.init()
    node = AmclNomotionUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
