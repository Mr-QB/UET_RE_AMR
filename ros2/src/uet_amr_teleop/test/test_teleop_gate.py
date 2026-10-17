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


def acquire(gate, on=True):
    from std_srvs.srv import SetBool
    return gate._on_test_source(SetBool.Request(data=on), SetBool.Response())


def test_test_source_cannot_be_selected_via_set_teleop(gate):
    res = call(gate, 'test')
    assert not res.success and res.active_source == 'none'


def test_test_source_only_acquired_from_none(gate):
    call(gate, 'joy')
    assert not acquire(gate).success
    call(gate, 'none')
    assert acquire(gate).success and gate._active == 'test'


def test_set_teleop_refused_while_test_active(gate):
    acquire(gate)
    for src in ('joy', 'none', 'keyboard'):
        res = call(gate, src)
        assert not res.success and res.active_source == 'test'


def test_test_commands_clamped_and_only_forwarded_when_active(gate):
    gate._on_test_twist(twist(0.1))
    assert gate.sent == []
    acquire(gate)
    msg = twist(0.9)
    msg.angular.z = -3.0
    gate._on_test_twist(msg)
    assert (gate.sent[-1].linear.x, gate.sent[-1].angular.z) == (0.2, -0.6)


def test_non_finite_test_command_releases_with_zero(gate):
    acquire(gate)
    gate._on_test_twist(twist(float('nan')))
    assert gate._active == 'none' and gate.sent[-1].linear.x == 0.0


def test_watchdog_drops_silent_test_source(gate):
    acquire(gate)
    gate._test_seen -= 1.0
    gate._test_watchdog()
    assert gate._active == 'none' and gate.sent[-1].linear.x == 0.0 and gate.sent[-1].angular.z == 0.0


def test_release_is_idempotent(gate):
    acquire(gate)
    assert acquire(gate, False).success
    assert acquire(gate, False).success and gate._active == 'none'
