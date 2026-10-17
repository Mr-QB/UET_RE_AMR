"""End-to-end: executor + real teleop gate + a fake plant over real DDS (isolated domain)."""
import base64
import math
import os
import threading
import time

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '97')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('uet_amr_msgs')
pytest.importorskip('controller_manager_msgs')

from controller_manager_msgs.msg import ControllerState  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from uet_amr_msgs.msg import HardwareStatus, MotionTestState  # noqa: E402
from uet_amr_msgs.srv import SetTeleop, StartMotionTest, StartVelocityStep, StopMotionTest  # noqa: E402
from uet_amr_teleop.teleop_gate import TeleopGate  # noqa: E402

from uet_amr_evaluation.motion.executor import MotionTestExecutor  # noqa: E402
from uet_amr_evaluation.motion.recorder import export_archive  # noqa: E402


class Plant(Node):
    """Fake drive: integrates the controller topic, publishes odometry, hw status and list_controllers."""

    def __init__(self):
        super().__init__('fake_plant')
        self.v = self.w = self.x = self.y = self.yaw = 0.0
        self.cmd = Twist()
        self.cmd_log = []
        self.fault = False
        self.controller_active = True
        self.odom_enabled = True
        self.create_subscription(Twist, '/diff_drive_controller/cmd_vel_unstamped', self._cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.wheel_pub = self.create_publisher(Odometry, '/diff_drive_controller/odom', 10)
        self.hw_pub = self.create_publisher(HardwareStatus, '/hardware/status', 10)
        self.create_service(ListControllers, '/controller_manager/list_controllers', self._list)
        self.create_timer(0.02, self._step)
        self.create_timer(0.1, self._hw)

    def _cmd(self, msg):
        self.cmd = msg
        self.cmd_log.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def _list(self, request, response):
        c = ControllerState(name='diff_drive_controller', state='active' if self.controller_active else 'inactive')
        response.controller = [c]
        return response

    def _hw(self):
        msg = HardwareStatus()
        msg.sys_status = HardwareStatus.SYS_FAULT if self.fault else HardwareStatus.SYS_MANUAL
        self.hw_pub.publish(msg)

    def _step(self):
        dt = 0.02
        self.v += max(-0.8 * dt, min(1.2 * dt, self.cmd.linear.x - self.v))
        self.w += max(-1.2 * dt, min(1.8 * dt, self.cmd.angular.z - self.w))
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        if not self.odom_enabled:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(msg)
        self.wheel_pub.publish(msg)


class Rig:
    def __init__(self, tmp):
        rclpy.init()
        self.plant = Plant()
        self.gate = TeleopGate()
        self.mgr = Node('fake_mode_manager', parameter_overrides=[
            rclpy.parameter.Parameter('motion_test.archive_dir', value=str(tmp))])
        self.lock = threading.Lock()
        self.mode = 'slam'
        self.exe = MotionTestExecutor(self.mgr, self.lock, lambda: self.mode)
        self.client = Node('test_client')
        self.states = []
        self.client.create_subscription(MotionTestState, '/amr/motion_test/state',
                                        lambda m: self.states.append(m), 100)
        self.hb_pub = self.client.create_publisher(String, '/amr/motion_test/heartbeat', 10)
        self.start_motion = self.client.create_client(StartMotionTest, '/amr/start_motion_test')
        self.start_step = self.client.create_client(StartVelocityStep, '/amr/start_velocity_step')
        self.stop = self.client.create_client(StopMotionTest, '/amr/stop_motion_test')
        self.set_teleop = self.client.create_client(SetTeleop, '/amr/set_teleop')
        self.hb_id, self._hb_stop = None, False
        self.client.create_timer(0.1, self._hb)
        self.executor = MultiThreadedExecutor(num_threads=8)
        for n in (self.plant, self.gate, self.mgr, self.client):
            self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _hb(self):
        if self.hb_id and not self._hb_stop:
            self.hb_pub.publish(String(data=self.hb_id))

    def call(self, client, req, timeout=5.0):
        assert client.wait_for_service(timeout_sec=5.0)
        fut = client.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        return fut.result()

    def wait_for(self, predicate, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def last(self):
        return self.states[-1] if self.states else None

    def ready(self):
        return self.wait_for(lambda: self.last() is not None and self.last().ready, 10.0)

    def close(self):
        self.exe.shutdown()
        self.executor.shutdown()
        for n in (self.plant, self.gate, self.mgr, self.client):
            n.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig(tmp_path):
    r = Rig(tmp_path)
    assert r.ready(), r.last()
    yield r
    r.close()


def start(rig, run_id, kind='linear', target=0.3, speed=0.2, hb=True):
    rig.hb_id, rig._hb_stop = (run_id if hb else None), False
    return rig.call(rig.start_motion, StartMotionTest.Request(run_id=run_id, kind=kind, target=target, speed_limit=speed))


def terminal(rig, run_id, timeout=40.0):
    ok = rig.wait_for(lambda: rig.last().run_id == run_id and rig.last().phase in ('completed', 'aborted', 'failed'),
                      timeout)
    assert ok, rig.last()
    return rig.last()


def assert_safe_exit(rig):
    assert rig.wait_for(lambda: rig.gate._active == 'none', 3.0)
    assert not rig.lock.locked()
    assert rig.wait_for(lambda: abs(rig.plant.v) < 1e-3, 5.0)
    assert rig.plant.cmd.linear.x == 0.0 and rig.plant.cmd.angular.z == 0.0
    assert rig.wait_for(lambda: rig.last().manual_source == 'none', 3.0)


def test_linear_run_end_to_end_and_archive_round_trip(rig, tmp_path):
    res = start(rig, 'run-lin', target=0.3)
    assert res.accepted, res.message
    final = terminal(rig, 'run-lin')
    assert final.phase == 'completed' and final.kind == 'linear'
    assert abs(final.progress - 0.3) <= 0.02
    assert final.header.frame_id == 'odom' and final.mode == 'slam'
    assert_safe_exit(rig)
    time.sleep(1.0)                                    # capture tail + worker
    seqs = [s.sequence for s in rig.states]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    archive = export_archive(str(tmp_path), 'run-lin')
    topics = {p['topic'] for p in archive['packets']}
    assert {'/odometry/filtered', '/diff_drive_controller/odom', '/amr/motion_test/state'} <= topics
    states = [deserialize_message(base64.b64decode(p['cdr_base64']), MotionTestState)
              for p in archive['packets'] if p['topic'] == '/amr/motion_test/state']
    assert states[0].phase == 'accepted' and states[-1].phase == 'completed'
    assert any(p['topic'] == '/hardware/status' for p in archive['packets'])
    # nothing moved before a post-acceptance heartbeat + baseline
    first_move = next(t for t, v, w in rig.plant.cmd_log if v != 0)
    first_cmd = rig.plant.cmd_log[0][0]
    assert first_move - first_cmd >= 0.45


def test_heartbeat_loss_aborts_and_restores_off(rig):
    assert start(rig, 'run-hb', target=1.5).accepted
    assert rig.wait_for(lambda: rig.last().phase == 'running', 10.0)
    rig._hb_stop = True
    final = terminal(rig, 'run-hb', 5.0)
    assert (final.phase, final.reason) == ('aborted', 'heartbeat_timeout')
    assert_safe_exit(rig)


def test_never_started_without_heartbeat(rig):
    assert start(rig, 'run-nohb', hb=False).accepted
    time.sleep(1.5)
    assert all(v == 0.0 and w == 0.0 for _, v, w in rig.plant.cmd_log)
    rig.hb_id = 'run-nohb'                              # late heartbeat is accepted and the run proceeds
    assert terminal(rig, 'run-nohb').phase == 'completed'


def test_ownership_rules(rig):
    assert start(rig, 'run-own', target=1.5).accepted
    assert rig.wait_for(lambda: rig.last().phase == 'running', 10.0)
    dup = start(rig, 'run-own')
    assert not dup.accepted and 'already' in dup.message
    rig.hb_id = 'run-own'
    other = start(rig, 'run-two')
    assert not other.accepted
    rig.hb_id = 'run-own'
    rejected_again = start(rig, 'run-two')
    assert not rejected_again.accepted                  # rejected IDs stay burned
    assert not rig.call(rig.set_teleop, SetTeleop.Request(source='joy')).success
    assert not rig.call(rig.stop, StopMotionTest.Request(run_id='someone-else')).accepted
    assert rig.last().phase == 'running'                # other-run Stop changed nothing
    assert rig.lock.locked()                            # mode/save would be refused by the shared lock
    assert rig.call(rig.stop, StopMotionTest.Request(run_id='run-own')).accepted
    final = terminal(rig, 'run-own', 5.0)
    assert (final.phase, final.reason) == ('aborted', 'stop_requested')
    assert rig.call(rig.stop, StopMotionTest.Request(run_id='run-own')).accepted      # idempotent
    assert_safe_exit(rig)
    assert not start(rig, 'run-own').accepted           # finished IDs stay burned too


@pytest.mark.parametrize('setup,why', [
    (lambda r: setattr(r, 'mode', 'nav'), 'Mapping'),
    (lambda r: setattr(r.plant, 'fault', True), 'fault'),
    (lambda r: setattr(r.plant, 'controller_active', False), 'controller'),
    (lambda r: r.call(r.set_teleop, SetTeleop.Request(source='joy')), 'manual'),
])
def test_start_preconditions(rig, setup, why):
    setup(rig)
    time.sleep(3.2)                                     # let status caches refresh
    res = start(rig, 'run-pre')
    assert not res.accepted and why.lower() in res.message.lower(), res.message
    assert not rig.lock.locked() and rig.gate._active != 'test'


def test_invalid_requests(rig):
    for i, (kind, target, speed) in enumerate([('linear', math.nan, 0.1), ('linear', 5.0, 0.1), ('rotation', 0.01, 0.3),
                                               ('spin', 1.0, 0.1), ('linear', 1.0, 0.5)]):
        assert not start(rig, f'bad-{i}', kind, target, speed).accepted
    assert not rig.lock.locked()
    assert not rig.call(rig.start_step, StartVelocityStep.Request(
        run_id='bad-step', axis='linear', target_velocity=0.2, duration_s=10.0)).accepted


def test_stale_odometry_aborts(rig):
    assert start(rig, 'run-stale', target=1.5).accepted
    assert rig.wait_for(lambda: rig.last().phase == 'running', 10.0)
    rig.plant.odom_enabled = False
    final = terminal(rig, 'run-stale', 5.0)
    assert (final.phase, final.reason) == ('aborted', 'stale_odometry')
    assert_safe_exit(rig)


def test_controller_loss_and_hardware_fault_abort(rig):
    assert start(rig, 'run-fault', target=1.5).accepted
    assert rig.wait_for(lambda: rig.last().phase == 'running', 10.0)
    rig.plant.fault = True
    final = terminal(rig, 'run-fault', 5.0)
    assert (final.phase, final.reason) == ('aborted', 'hardware_fault')
    assert_safe_exit(rig)


def test_velocity_step_end_to_end(rig, tmp_path):
    rig.hb_id = 'step-1'
    res = rig.call(rig.start_step, StartVelocityStep.Request(
        run_id='step-1', axis='angular', target_velocity=-0.4, duration_s=2.0))
    assert res.accepted, res.message
    final = terminal(rig, 'step-1')
    assert final.phase == 'completed' and final.kind == 'angular_velocity' and final.target == -0.4
    assert final.progress < -0.5
    assert_safe_exit(rig)
    time.sleep(1.0)
    running = [s for s in rig.states if s.run_id == 'step-1' and s.phase == 'running']
    assert {round(s.commanded_angular, 6) for s in running} == {-0.4}
    archive = export_archive(str(tmp_path), 'step-1')
    assert archive['expected_count'] == len(archive['packets']) > 50
