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
Removes nvblox obstacle cells that lie inside the robot's own footprint from the 2D distance slice the local costmap reads.

The robot occupies its footprint, so nothing real can be there. Stale or smeared cells (wheel-odometry slip while stuck on the
door threshold integrates depth at wrong poses) can end up under the robot, and the controller then sees its own footprint in
collision and refuses to move. Cells inside the rectangle are set to "far from any obstacle"; everything outside is untouched, so
real obstacles beside, behind or ahead of the footprint still count.
"""
import copy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nvblox_msgs.msg import DistanceMapSlice


class NvbloxFootprintFilter(Node):
    def __init__(self):
        super().__init__('nvblox_footprint_filter')
        self.declare_parameter('input', '/nvblox_node/static_map_slice')
        self.declare_parameter('output', '/nvblox_node/static_map_slice_clean')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('front', 0.34)    # metres from base_footprint: footprint 0.29 + 5 cm
        self.declare_parameter('rear', 0.34)
        self.declare_parameter('half_width', 0.31)  # 0.26 + 5 cm
        self.declare_parameter('clear_value', 2.0)  # distance (m) written into cleared cells: beyond max_obstacle_distance
        p = self.get_parameter
        self.base = p('base_frame').value
        self.front, self.rear, self.hw = p('front').value, p('rear').value, p('half_width').value
        self.clear = p('clear_value').value
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(DistanceMapSlice, p('output').value, 10)
        self.create_subscription(DistanceMapSlice, p('input').value, self.on_slice, 10)
        self.get_logger().info('%s -> %s, clearing the footprint rectangle' % (p('input').value, p('output').value))

    def on_slice(self, msg):
        out = copy.copy(msg)
        try:
            T = self.tf.lookup_transform(msg.header.frame_id, self.base, Time())
        except Exception:
            self.pub.publish(msg)   # no pose yet: pass through unchanged
            return
        t, q = T.transform.translation, T.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        D = np.array(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        r = msg.resolution
        # Only look at the bounding box of the rectangle, not the whole slice.
        reach = max(self.front, self.rear, self.hw) * 1.5
        i0 = max(int((t.x - reach - msg.origin.x) / r), 0)
        i1 = min(int((t.x + reach - msg.origin.x) / r) + 1, msg.width)
        j0 = max(int((t.y - reach - msg.origin.y) / r), 0)
        j1 = min(int((t.y + reach - msg.origin.y) / r) + 1, msg.height)
        if i1 > i0 and j1 > j0:
            ii, jj = np.meshgrid(np.arange(i0, i1), np.arange(j0, j1))
            dx = msg.origin.x + (ii + 0.5) * r - t.x
            dy = msg.origin.y + (jj + 0.5) * r - t.y
            fx = dx * math.cos(yaw) + dy * math.sin(yaw)
            ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
            inside = (fx < self.front) & (fx > -self.rear) & (np.abs(ly) < self.hw)
            window = D[j0:j1, i0:i1]
            known = window != msg.unknown_value
            window[inside & known] = self.clear
        out.data = D.reshape(-1).tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = NvbloxFootprintFilter()
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
