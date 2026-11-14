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

from geometry_msgs.msg import Twist
import pytest
import rclpy
from uet_amr_msgs.srv import SetTeleop

from uet_amr_teleop.teleop_gate import TeleopGate


@pytest.fixture
def gate():
    rclpy.init()
    node = TeleopGate()
    sent = []
    # Replace the real publishers so the test can never reach a controller.
    node._pub = type('Pub', (), {'publish': lambda self, m: sent.append(m)})()
    node.sent = sent
    yield node
    node.destroy_node()
    rclpy.shutdown()


def call(gate, source):
    return gate._on_set_teleop(SetTeleop.Request(source=source), SetTeleop.Response())


def twist(x):
    msg = Twist()
    msg.linear.x = x
    return msg


def test_starts_disabled(gate):
    gate._on_twist('joy', twist(1.0))
    assert gate.sent == []


def test_forwards_only_active_source(gate):
    assert call(gate, 'joy').success
    gate._on_twist('joy', twist(0.2))
    gate._on_twist('keyboard', twist(-0.2))
    assert [m.linear.x for m in gate.sent] == [0.2]


def test_switching_away_sends_stop(gate):
    call(gate, 'joy')
    call(gate, 'keyboard')
    assert [m.linear.x for m in gate.sent] == [0.0]
    call(gate, 'none')
    assert [m.linear.x for m in gate.sent] == [0.0, 0.0]
    gate._on_twist('keyboard', twist(0.5))
    assert len(gate.sent) == 2


def test_unknown_source_rejected_and_state_kept(gate):
    call(gate, 'joy')
    res = call(gate, 'bogus')
    assert not res.success
    assert res.active_source == 'joy'


def test_source_name_is_normalised(gate):
    assert call(gate, ' JOY ').active_source == 'joy'
