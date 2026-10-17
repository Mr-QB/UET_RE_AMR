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
Gate between the teleop sources and the drive controller.

Each source publishes Twists on /teleop/<source>/cmd_vel_raw. Only the active
source is forwarded to `output_topic`; select it with the
uet_amr_msgs/srv/SetTeleop service /amr/set_teleop:

    ros2 service call /amr/set_teleop uet_amr_msgs/srv/SetTeleop "{source: keyboard}"

"none" forwards nothing, so another publisher (e.g. Nav2) can drive. Switching
away from a source sends one zero Twist so the robot stops. The active source is
also published (latched) on /amr/teleop_source for UIs.

A fourth source, "test", belongs to the motion-test executor (uet_amr_evaluation). It cannot be chosen through
/amr/set_teleop: only the internal service /amr/internal/test_source (std_srvs/SetBool) acquires it, and only while the
source is "none". While it is held, /amr/set_teleop is refused. Test commands are clamped to the executor's hard limits,
and a watchdog drops the source (zero command, back to "none") if /teleop/test/cmd_vel_raw goes quiet, so a dead
executor cannot leave the robot driving.
"""
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import SetBool
from std_msgs.msg import String
from uet_amr_msgs.srv import SetTeleop

NONE = 'none'
TEST = 'test'
TEST_MAX_LINEAR = 0.2
TEST_MAX_ANGULAR = 0.6


class TeleopGate(Node):

    def __init__(self):
        super().__init__('teleop_gate')
        self.declare_parameter('sources', ['joy', 'keyboard'])
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('initial_source', NONE)
        self.declare_parameter('test_timeout', 0.5)

        self._sources = list(self.get_parameter('sources').value)
        self._active = self.get_parameter('initial_source').value
        if self._active != NONE and self._active not in self._sources:
            self.get_logger().warn(f'unknown initial_source "{self._active}", using none')
            self._active = NONE

        self._pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        for source in self._sources:
            self.create_subscription(
                Twist, f'/teleop/{source}/cmd_vel_raw',
                lambda msg, source=source: self._on_twist(source, msg), 10)
        self._test_timeout = float(self.get_parameter('test_timeout').value)
        self._test_seen = 0.0
        self.create_subscription(Twist, f'/teleop/{TEST}/cmd_vel_raw', self._on_test_twist, 10)
        self.create_service(SetBool, '/amr/internal/test_source', self._on_test_source)
        self.create_timer(0.05, self._test_watchdog)
        self._source_pub = self.create_publisher(
            String, '/amr/teleop_source',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._publish_source()
        self.create_service(SetTeleop, '/amr/set_teleop', self._on_set_teleop)
        self.get_logger().info(
            f'teleop source: {self._active} (available: {", ".join(self._sources)})')

    def _publish_source(self):
        self._source_pub.publish(String(data=self._active))

    def _on_twist(self, source, msg):
        if source == self._active:
            self._pub.publish(msg)

    def _on_test_twist(self, msg):
        if self._active != TEST:
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self._release_test('non-finite test command')
            return
        out = Twist()
        out.linear.x = max(-TEST_MAX_LINEAR, min(TEST_MAX_LINEAR, msg.linear.x))
        out.angular.z = max(-TEST_MAX_ANGULAR, min(TEST_MAX_ANGULAR, msg.angular.z))
        self._test_seen = time.monotonic()
        self._pub.publish(out)

    def _release_test(self, why):
        if self._active == TEST:
            self._pub.publish(Twist())
            self._active = NONE
            self._publish_source()
            self.get_logger().info(f'test source released ({why})')

    def _test_watchdog(self):
        if self._active == TEST and time.monotonic() - self._test_seen > self._test_timeout:
            self._release_test('no test command within timeout')

    def _on_test_source(self, request, response):
        if request.data:
            if self._active != NONE:
                response.success = False
                response.message = f'source is "{self._active}", not "none"'
            else:
                self._active = TEST
                self._test_seen = time.monotonic()
                self._publish_source()
                response.success = True
                response.message = 'test source acquired'
        else:
            self._release_test('released by executor')
            response.success = True
            response.message = f'teleop source: {self._active}'
        return response

    def _on_set_teleop(self, request, response):
        source = request.source.strip().lower()
        if self._active == TEST:
            response.success = False
            response.message = 'a motion test owns the drive controller'
        elif source != NONE and source not in self._sources:
            response.success = False
            response.message = f'unknown source "{request.source}"; use none or one of {self._sources}'
        else:
            if source != self._active and self._active != NONE:
                self._pub.publish(Twist())
            self._active = source
            self._publish_source()
            response.success = True
            response.message = f'teleop source: {source}'
            self.get_logger().info(response.message)
        response.active_source = self._active
        return response


def main():
    rclpy.init()
    node = TeleopGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
