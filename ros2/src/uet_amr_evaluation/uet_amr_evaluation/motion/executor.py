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
ROS side of the motion-test executor (web "motion test" and "velocity step" features).

Lives inside amr_mode_manager so that it shares that node's operation lock: while a test runs, /amr/set_mode and
/amr/save_map are refused by the same non-blocking lock they already use. The state machine itself is in core.py; this
module gathers inputs, publishes commands (via the teleop gate's private "test" source) and the 20 Hz state topic, runs
the heartbeat watchdog, starts/stops the local recorder, and guarantees the exit sequence on every path:
zero command -> release test source (Off) -> release lock -> terminal state.

Services:   /amr/start_motion_test, /amr/stop_motion_test, /amr/start_velocity_step
Topics:     /amr/motion_test/state (20 Hz out), /amr/motion_test/heartbeat (in, exact run ID every 100 ms)
Recorded:   /odometry/filtered, /diff_drive_controller/odom, /hardware/status, /amr/motion_test/state
"""
import math
import os
import threading
import time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uet_amr_msgs.msg import HardwareStatus, MotionTestState
from uet_amr_msgs.srv import StartMotionTest, StartVelocityStep, StopMotionTest

from . import core
from .core import Inputs, MotionTestCore, Odom
from .recorder import RunRecorder, run_exists, valid_run_id

STATE_TOPIC = '/amr/motion_test/state'
HEARTBEAT_TOPIC = '/amr/motion_test/heartbeat'
TEST_CMD_TOPIC = '/teleop/test/cmd_vel_raw'
TEST_SOURCE_SERVICE = '/amr/internal/test_source'
ODOM_TOPIC = '/odometry/filtered'
WHEEL_ODOM_TOPIC = '/diff_drive_controller/odom'
HW_TOPIC = '/hardware/status'
RECORDED = [ODOM_TOPIC, WHEEL_ODOM_TOPIC, HW_TOPIC, STATE_TOPIC]
CONTROLLER_NAME = 'diff_drive_controller'
CONTROLLER_STALE_S = 2.5
HW_STALE_S = 5.0
CAPTURE_TAIL_S = 0.5
SNAPSHOT_PARAMS = {
    '/diff_drive_controller': [
        'wheel_separation', 'wheel_radius', 'left_wheel_radius_multiplier', 'right_wheel_radius_multiplier',
        'cmd_vel_timeout', 'publish_rate', 'open_loop',
        'linear.x.max_velocity', 'linear.x.max_acceleration', 'linear.x.max_deceleration',
        'linear.x.has_acceleration_limits', 'linear.x.has_deceleration_limits',
        'angular.z.max_velocity', 'angular.z.max_acceleration', 'angular.z.max_deceleration',
        'angular.z.has_acceleration_limits', 'angular.z.has_deceleration_limits'],
    '/ekf_filter_node': ['frequency', 'two_d_mode', 'sensor_timeout', 'odom0', 'odom0_config', 'imu0', 'imu0_config'],
}


class _Run:
    def __init__(self, core_, recorder, t_accept):
        self.core, self.recorder, self.t_accept = core_, recorder, t_accept
        self.heartbeat_at = None
        self.stop_requested = False
        self.capture_end_at = None


class MotionTestExecutor:

    def __init__(self, node, op_lock, get_mode):
        """op_lock: the mode manager's non-blocking operation lock. get_mode(): 'slam' | 'nav' | 'stopped'."""
        self._node, self._op_lock, self._get_mode = node, op_lock, get_mode
        node.declare_parameter('motion_test.archive_dir', os.path.expanduser('~/amr_motion_tests'))
        node.declare_parameter('motion_test.controller_topic', '/diff_drive_controller/cmd_vel_unstamped')
        node.declare_parameter('motion_test.allowed_controller_publishers', ['teleop_gate'])
        self._dir = os.path.abspath(os.path.expanduser(node.get_parameter('motion_test.archive_dir').value))
        self._controller_topic = node.get_parameter('motion_test.controller_topic').value
        self._allowed_publishers = set(node.get_parameter('motion_test.allowed_controller_publishers').value)

        self._mutex = threading.RLock()          # guards everything below
        self._start_lock = threading.Lock()      # serialises Start requests (they block on other nodes)
        self._seen_ids = set()                   # every Start ID ever seen, rejected ones included
        self._finished_ids = set()
        self._run = None
        self._capturing = ()                     # recorders still accepting samples (run + capture tail)
        self._ending = []                        # finished runs waiting out the capture tail
        self._last = None                        # terminal snapshot, republished until the next start
        self._sequence = 0
        self._odom = None                        # (Odom, recv steady s, header stamp ns, frame_id)
        self._hw = None                          # (fault, recv steady s)
        self._source = None                      # last /amr/teleop_source
        self._controller = (False, 0.0)          # (active, checked steady s)
        self._release_pending = False
        self._release_sent = 0.0

        subs, srv, ctl = ReentrantCallbackGroup(), ReentrantCallbackGroup(), MutuallyExclusiveCallbackGroup()
        reliable = QoSProfile(depth=10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = node.create_publisher(MotionTestState, STATE_TOPIC, reliable)
        self._cmd_pub = node.create_publisher(Twist, TEST_CMD_TOPIC, reliable)
        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, QoSProfile(depth=10), callback_group=subs)
        node.create_subscription(HardwareStatus, HW_TOPIC, self._on_hw, reliable, callback_group=subs)
        node.create_subscription(String, '/amr/teleop_source', self._on_source, latched, callback_group=subs)
        node.create_subscription(String, HEARTBEAT_TOPIC, self._on_heartbeat, reliable, callback_group=subs)
        for topic, msg_type in ((ODOM_TOPIC, Odometry), (WHEEL_ODOM_TOPIC, Odometry),
                                (HW_TOPIC, HardwareStatus), (STATE_TOPIC, MotionTestState)):
            node.create_subscription(msg_type, topic, lambda raw, t=topic: self._on_raw(t, raw),
                                     QoSProfile(depth=100), callback_group=subs, raw=True)
        self._source_client = node.create_client(SetBool, TEST_SOURCE_SERVICE, callback_group=srv)
        self._controllers_client = node.create_client(ListControllers, '/controller_manager/list_controllers',
                                                      callback_group=srv)
        self._param_clients = {name: node.create_client(GetParameters, name + '/get_parameters', callback_group=srv)
                               for name in SNAPSHOT_PARAMS}
        node.create_service(StartMotionTest, '/amr/start_motion_test', self._on_start_motion, callback_group=srv)
        node.create_service(StartVelocityStep, '/amr/start_velocity_step', self._on_start_step, callback_group=srv)
        node.create_service(StopMotionTest, '/amr/stop_motion_test', self._on_stop, callback_group=srv)
        node.create_timer(core.DT, self._tick, callback_group=ctl)
        node.create_timer(0.5, self._poll_controller, callback_group=srv)
        node.get_logger().info(
            f'Motion test ready: /amr/start_motion_test, /amr/start_velocity_step, /amr/stop_motion_test '
            f'(captures in {self._dir})')

    # ------------------------------------------------------------------ subscriptions
    def _on_raw(self, topic, raw):
        for recorder in self._capturing:
            recorder.record(topic, raw)

    def _on_odom(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        odom = Odom(stamp_ns, p.x, p.y, yaw, t.linear.x, t.angular.z)
        with self._mutex:
            self._odom = (odom, time.monotonic(), msg.header.stamp, msg.header.frame_id)

    def _on_hw(self, msg):
        with self._mutex:
            self._hw = (msg.sys_status == HardwareStatus.SYS_FAULT, time.monotonic())

    def _on_source(self, msg):
        with self._mutex:
            self._source = msg.data

    def _on_heartbeat(self, msg):
        with self._mutex:
            run = self._run
            if run is not None and msg.data == run.core.run_id and not run.core.terminal:
                run.heartbeat_at = time.monotonic()      # ids of finished / unknown runs are ignored

    def _poll_controller(self):
        if not self._controllers_client.service_is_ready():
            return
        future = self._controllers_client.call_async(ListControllers.Request())

        def done(f):
            try:
                active = any(c.name == CONTROLLER_NAME and c.state == 'active' for c in f.result().controller)
            except Exception:
                active = False
            with self._mutex:
                self._controller = (active, time.monotonic())
        future.add_done_callback(done)

    # ------------------------------------------------------------------ helpers (call with _mutex held)
    def _odom_age(self, now):
        if self._odom is None:
            return math.inf
        _, recv, stamp, frame = self._odom
        if frame != 'odom':
            return math.inf
        stamp_age = (self._node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        if stamp_age < -core.ODOM_MAX_AGE_S:
            return math.inf
        return max(now - recv, stamp_age, 0.0)

    def _controller_active(self, now):
        active, checked = self._controller
        return active and now - checked <= CONTROLLER_STALE_S

    def _hw_fault(self, now):
        return self._hw is not None and self._hw[0]

    def _not_ready_reason(self, now, lock_held=False):
        mode = self._get_mode()
        if mode != 'slam':
            return f'robot is in {mode} mode; Mapping mode is required'
        if self._run is not None or (self._op_lock.locked() and not lock_held):
            return 'a test, mode switch or map save is in progress'
        if self._source != 'none':
            return f'manual input is "{self._source}"; it must be Off'
        if not self._controller_active(now):
            return 'drive controller is not active'
        if self._odom_age(now) > core.ODOM_MAX_AGE_S:
            return 'odometry is stale'
        if self._hw is None or now - self._hw[1] > HW_STALE_S:
            return 'no hardware status'
        if self._hw[0]:
            return 'hardware fault'
        return ''

    # ------------------------------------------------------------------ services
    def _on_start_motion(self, request, response):
        def validate():
            err, speed = core.validate_motion(request.kind, request.target, request.speed_limit)
            return err, (lambda t: MotionTestCore(request.run_id, request.kind, request.target, speed, t))
        response.accepted, response.message = self._start(request.run_id, validate)
        return response

    def _on_start_step(self, request, response):
        def validate():
            err = core.validate_step(request.axis, request.target_velocity, request.duration_s)
            return err, (lambda t: MotionTestCore.for_step(
                request.run_id, request.axis, request.target_velocity, request.duration_s, t))
        response.accepted, response.message = self._start(request.run_id, validate)
        return response

    def _on_stop(self, request, response):
        with self._mutex:
            run = self._run
            if run is not None and run.core.run_id == request.run_id:
                self._abort(run, 'stop_requested')
                response.accepted, response.message = True, 'stopped'
            elif request.run_id in self._finished_ids:
                response.accepted, response.message = True, 'run already finished'
            else:
                response.accepted, response.message = False, 'not the active run; nothing changed'
        return response

    def _start(self, run_id, validate):
        with self._start_lock:
            if not isinstance(run_id, str) or not run_id:
                return False, 'run_id is required'
            with self._mutex:
                if run_id in self._seen_ids:
                    return False, f'run_id {run_id!r} was already used'
                self._seen_ids.add(run_id)
            if not valid_run_id(run_id):
                return False, 'run_id may use letters, digits, "_", "-", "." (max 128 chars)'
            if run_exists(self._dir, run_id):
                return False, f'a capture for {run_id!r} already exists on the robot'
            error, build = validate()
            if error:
                return False, error
            if not self._op_lock.acquire(blocking=False):
                return False, 'a mode switch, map save or test is already in progress'
            owns_source = accepted = False
            try:
                with self._mutex:
                    reason = self._not_ready_reason(time.monotonic(), lock_held=True)
                if reason:
                    return False, reason
                publishers = {i.node_name for i in self._node.get_publishers_info_by_topic(self._controller_topic)}
                extra = publishers - self._allowed_publishers
                if extra:
                    return False, f'other publishers can drive the controller: {sorted(extra)}'
                snapshot = self._parameter_snapshot()
                if not self._set_test_source(True):
                    return False, 'teleop gate refused or did not answer the test source request'
                owns_source = True
                t_accept = time.monotonic()
                try:
                    recorder = RunRecorder(self._dir, run_id, RECORDED, snapshot=snapshot)
                except OSError as exc:
                    return False, f'cannot start local recording: {exc}'
                with self._mutex:
                    run = _Run(build(t_accept), recorder, t_accept)
                    self._run, self._last = run, None
                    self._capturing = self._capturing + (recorder,)
                    self._source = 'test'
                    self._publish_state(time.monotonic())
                owns_source, accepted = False, True     # now owned by the run; _finish releases both
                return True, f'accepted; send heartbeats for {run_id!r} every 100 ms'
            finally:
                if owns_source:
                    self._set_test_source(False, wait=False)
                if not accepted:
                    self._op_lock.release()

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        return future.result() if future.done() and future.exception() is None else None

    def _set_test_source(self, acquire, wait=True):
        if not self._source_client.wait_for_service(timeout_sec=1.0 if wait else 0.0):
            return False
        future = self._source_client.call_async(SetBool.Request(data=acquire))
        if not wait:
            return True
        result = self._wait(future, 1.0)
        return bool(result and result.success)

    def _parameter_snapshot(self):
        """Best-effort record of acceleration limits and filter settings the response depends on."""
        snapshot = {'executor': {
            'command_ramp_accel_decel': core.RAMP, 'plan_decel': core.PLAN_DECEL, 'tolerance_linear_m': core.LINEAR_TOL,
            'tolerance_angular_rad': core.ANGULAR_TOL, 'timeout_s': core.TIMEOUT_S}}
        futures = {}
        for node_name, names in SNAPSHOT_PARAMS.items():
            client = self._param_clients[node_name]
            if client.wait_for_service(timeout_sec=0.2):
                futures[node_name] = (names, client.call_async(GetParameters.Request(names=names)))
        for node_name, (names, future) in futures.items():
            result = self._wait(future, 1.0)
            if result is None:
                snapshot[node_name] = {'error': 'no answer'}
                continue
            values = {}
            for name, v in zip(names, result.values):
                values[name] = {
                    ParameterType.PARAMETER_BOOL: v.bool_value, ParameterType.PARAMETER_INTEGER: v.integer_value,
                    ParameterType.PARAMETER_DOUBLE: v.double_value, ParameterType.PARAMETER_STRING: v.string_value,
                    ParameterType.PARAMETER_BOOL_ARRAY: list(v.bool_array_value),
                    ParameterType.PARAMETER_DOUBLE_ARRAY: list(v.double_array_value),
                    ParameterType.PARAMETER_INTEGER_ARRAY: list(v.integer_array_value),
                    ParameterType.PARAMETER_STRING_ARRAY: list(v.string_array_value),
                }.get(v.type)
            snapshot[node_name] = values
        return snapshot

    # ------------------------------------------------------------------ control loop (20 Hz)
    def _tick(self):
        now = time.monotonic()
        with self._mutex:
            try:
                run = self._run
                if run is not None:
                    self._step_run(run, now)
                self._housekeeping(now)
            except Exception as exc:                       # never leave a run driving after a bug
                self._node.get_logger().error(f'motion test loop error: {exc!r}')
                if self._run is not None:
                    self._abort(self._run, f'internal_error: {exc!r}', failed=True)
            self._publish_state(now)

    def _step_run(self, run, now):
        odom = self._odom[0] if self._odom else None
        inp = Inputs(
            now=now, odom=odom, odom_age=self._odom_age(now), heartbeat_at=run.heartbeat_at,
            hw_fault=self._hw_fault(now), controller_active=self._controller_active(now),
            ownership_ok=self._source == 'test' or now - run.t_accept < 1.0,
            mode_ok=self._get_mode() == 'slam', stop_requested=run.stop_requested)
        lin, ang = run.core.step(inp)
        self._publish_cmd(lin, ang)
        if run.core.terminal:
            self._finish(run)

    def _abort(self, run, reason, failed=False):
        run.core._finish(core.FAILED if failed else core.ABORTED, reason)
        self._finish(run)

    def _publish_cmd(self, lin, ang):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(lin), float(ang)
        self._cmd_pub.publish(msg)

    def _finish(self, run):
        """Every exit: zero command, Off, release lock, terminal state."""
        self._publish_cmd(0.0, 0.0)
        self._set_test_source(False, wait=False)
        self._release_pending, self._release_sent = True, time.monotonic()
        self._last = run.core
        self._finished_ids.add(run.core.run_id)
        run.capture_end_at = time.monotonic() + CAPTURE_TAIL_S
        self._ending.append(run)
        self._run = None
        if self._op_lock.locked():
            self._op_lock.release()
        self._node.get_logger().info(f'motion test {run.core.run_id}: {run.core.phase} ({run.core.reason})')

    def _housekeeping(self, now):
        # keep asking the gate to drop the test source until /amr/teleop_source confirms Off
        if self._release_pending:
            if self._source == 'none':
                self._release_pending = False
            elif now - self._release_sent > 0.25:
                self._set_test_source(False, wait=False)
                self._release_sent = now
        for run in list(self._ending):
            if now >= run.capture_end_at:
                self._capturing = tuple(r for r in self._capturing if r is not run.recorder)
                run.recorder.finish()                      # non-blocking; the disk worker drains and closes
                self._ending.remove(run)

    # ------------------------------------------------------------------ state topic
    def _publish_state(self, now):
        msg = MotionTestState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        self._sequence += 1
        msg.sequence = self._sequence
        c = self._run.core if self._run is not None else self._last
        mode = self._get_mode()
        msg.mode = mode if mode in ('slam', 'nav') else 'stopped'
        source = self._source or 'none'
        msg.manual_source = source if source in ('none', 'test', 'joy', 'keyboard') else 'none'
        if c is None:
            reason = self._not_ready_reason(now)
            msg.phase, msg.reason, msg.ready = 'idle', reason, reason == ''
        else:
            msg.run_id, msg.phase, msg.reason = c.run_id, c.phase, c.reason
            msg.kind, msg.target, msg.progress = c.kind, float(c.target), float(c.progress)
            msg.lateral_error, msg.heading_error = float(c.lateral), float(c.heading)
            msg.commanded_linear, msg.commanded_angular = (float(x) for x in c.command)
            msg.ready = False
            if self._run is None:
                reason = self._not_ready_reason(now)
                msg.ready = reason == ''
        if self._odom is not None:
            odom, _, stamp, _ = self._odom
            msg.measured_linear, msg.measured_angular = odom.v, odom.w
            msg.measurement_stamp = stamp
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------ shutdown
    def shutdown(self):
        with self._mutex:
            if self._run is not None:
                self._abort(self._run, 'shutdown')
            ending, self._ending = list(self._ending), []
            self._capturing = ()
            for run in ending:
                run.recorder.finish()
        for run in ending:
            run.recorder.join(2.0)
