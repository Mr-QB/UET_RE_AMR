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
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from uet_amr_msgs.srv import SetTeleop

NONE = 'none'


class TeleopGate(Node):

    def __init__(self):
        super().__init__('teleop_gate')
        self.declare_parameter('sources', ['joy', 'keyboard'])
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('initial_source', NONE)

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

    def _on_set_teleop(self, request, response):
        source = request.source.strip().lower()
        if source != NONE and source not in self._sources:
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
