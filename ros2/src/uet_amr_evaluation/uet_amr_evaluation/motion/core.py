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
Motion-test state machine (no ROS imports, so it can be driven with fake odometry in unit tests).

One MotionTestCore instance is one run. The ROS executor calls step() at 20 Hz with the current inputs and publishes
whatever command comes back. Every exit path goes through _finish(), which zeroes the command; the executor then
restores the teleop source and releases the operation lock.

Phases: accepted -> running -> settling -> completed, or aborted / failed from anywhere.
  aborted: a contract abort condition (stale odometry, heartbeat expiry, stop, hardware fault, controller or ownership
           loss, mode loss, timeout)
  failed:  the test itself could not be carried out (travel limit, robot would not hold still, internal error)
"""
import math
from dataclasses import dataclass
from typing import Optional

DT = 0.05
TWO_PI = 2.0 * math.pi

LINEAR_DEFAULT, ANGULAR_DEFAULT = 0.1, 0.3
LINEAR_MAX, ANGULAR_MAX = 0.2, 0.6
DISTANCE_RANGE = (0.05, 2.0)
ANGLE_RANGE = (math.radians(5.0), TWO_PI)
STEP_DURATION_RANGE = (1.0, 10.0)
STEP_PLATEAU_MAX = (1.8, TWO_PI - 0.5)

TIMEOUT_S = 60.0
ODOM_MAX_AGE_S = 0.25
HEARTBEAT_EXPIRY_S = 0.5
ACCEPT_TIMEOUT_S = 5.0     # first heartbeat + stationary baseline must be established within this
BASELINE_S = 0.5
SETTLE_S = 0.5
LINEAR_TOL, ANGULAR_TOL = 0.02, math.radians(2.0)
STILL_LINEAR, STILL_ANGULAR = 0.01, 0.03        # "near-zero velocity" for settling
BASELINE_STILL_LINEAR, BASELINE_STILL_ANGULAR = 0.02, 0.05
CREEP = {'linear': 0.03, 'rotation': 0.08}      # minimum speed while still outside the tolerance
PATH_MARGIN = {'linear': 0.25, 'rotation': 0.35}

# Command ramp (accel, decel) and planning deceleration; all below the controller's own limits
# (controllers.yaml: 1.2/0.8 m/s^2 and 1.8/1.2 rad/s^2) so the controller never has to clip the command.
RAMP = {'linear': (0.3, 0.6), 'rotation': (0.8, 1.0)}
PLAN_DECEL = {'linear': 0.25, 'rotation': 0.6}

ACCEPTED, RUNNING, SETTLING = 'accepted', 'running', 'settling'
COMPLETED, ABORTED, FAILED = 'completed', 'aborted', 'failed'
TERMINAL = (COMPLETED, ABORTED, FAILED)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def validate_motion(kind, target, speed_limit):
    """Return (error or None, effective speed limit)."""
    if kind not in ('linear', 'rotation'):
        return f'kind must be "linear" or "rotation", got {kind!r}', 0.0
    if not _finite(target, speed_limit):
        return 'target and speed_limit must be finite', 0.0
    lo, hi = DISTANCE_RANGE if kind == 'linear' else ANGLE_RANGE
    unit = 'm' if kind == 'linear' else 'rad'
    if not (lo - 1e-9 <= abs(target) <= hi + 1e-9):
        return f'|target| must be {lo:.4g}..{hi:.4g} {unit}', 0.0
    default, limit = (LINEAR_DEFAULT, LINEAR_MAX) if kind == 'linear' else (ANGULAR_DEFAULT, ANGULAR_MAX)
    if speed_limit < 0.0:
        return 'speed_limit must be positive', 0.0
    speed = default if speed_limit == 0.0 else speed_limit
    if speed > limit + 1e-9:
        return f'speed_limit exceeds the hard limit of {limit} {unit}/s', 0.0
    return None, speed


def validate_step(axis, velocity, duration):
    if axis not in ('linear', 'angular'):
        return f'axis must be "linear" or "angular", got {axis!r}'
    if not _finite(velocity, duration):
        return 'target_velocity and duration_s must be finite'
    limit = LINEAR_MAX if axis == 'linear' else ANGULAR_MAX
    if velocity == 0.0 or abs(velocity) > limit + 1e-9:
        return f'target_velocity must be nonzero with |v| <= {limit}'
    if not (STEP_DURATION_RANGE[0] <= duration <= STEP_DURATION_RANGE[1]):
        return 'duration_s must be 1..10 s'
    plateau_max = STEP_PLATEAU_MAX[0 if axis == 'linear' else 1]
    if abs(velocity) * duration > plateau_max + 1e-9:
        return f'plateau travel {abs(velocity) * duration:.3f} exceeds {plateau_max:.3f}'
    return None


@dataclass
class Odom:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    v: float          # twist.linear.x
    w: float          # twist.angular.z


@dataclass
class Inputs:
    now: float                          # steady (monotonic) seconds
    odom: Optional[Odom]
    odom_age: float = 0.0               # worst of receipt age and header-stamp age, seconds
    heartbeat_at: Optional[float] = None    # steady time of the last valid heartbeat received after acceptance
    hw_fault: bool = False
    controller_active: bool = True
    ownership_ok: bool = True           # test source still selected on the gate
    mode_ok: bool = True                # robot-confirmed mapping mode
    stop_requested: bool = False


class MotionTestCore:

    def __init__(self, run_id, kind, target, speed, t_accept):
        """kind: linear | rotation | linear_velocity | angular_velocity (target = signed velocity for steps)."""
        self.run_id, self.kind, self.target = run_id, kind, target
        self.is_step = kind.endswith('_velocity')
        self.axis = 'rotation' if kind in ('rotation', 'angular_velocity') else 'linear'
        self.speed = speed                  # speed limit (tests) / unused (steps)
        self.duration = 0.0                 # steps only
        self.t_accept = t_accept
        self.phase, self.reason = ACCEPTED, ''
        self.cmd = 0.0                      # commanded velocity on the active axis
        self.progress = self.lateral = self.heading = 0.0
        self.path = 0.0
        self.measured_v = self.measured_w = 0.0
        self._origin = None                 # (x, y, yaw) at the start of motion
        self._prev = None                   # (x, y, yaw) at the last ingested sample
        self._last_stamp = None
        self._baseline_since = None
        self._plateau_start = None
        self._zero_since = None
        self._still_since = None

    @classmethod
    def for_step(cls, run_id, axis, velocity, duration, t_accept):
        core = cls(run_id, f'{axis}_velocity', velocity, 0.0, t_accept)
        core.duration = duration
        return core

    @property
    def terminal(self):
        return self.phase in TERMINAL

    @property
    def command(self):
        """(linear, angular) setpoint to publish."""
        return (0.0, self.cmd) if self.axis == 'rotation' else (self.cmd, 0.0)

    def step(self, inp):
        if self.terminal:
            return self.command
        try:
            return self._step(inp)
        except Exception as exc:  # any bug must still end in a zero command
            self._finish(FAILED, f'internal_error: {exc!r}')
            return self.command

    # ------------------------------------------------------------------ internals
    def _finish(self, phase, reason):
        self.phase, self.reason, self.cmd = phase, reason, 0.0

    def _abort_reason(self, inp):
        if inp.stop_requested:
            return 'stop_requested'
        if not inp.mode_ok:
            return 'mode_lost'
        if not inp.ownership_ok:
            return 'ownership_lost'
        if not inp.controller_active:
            return 'controller_inactive'
        if inp.hw_fault:
            return 'hardware_fault'
        if inp.odom is None or inp.odom_age > ODOM_MAX_AGE_S:
            return 'stale_odometry'
        if inp.now - self.t_accept > TIMEOUT_S:
            return 'timeout'
        if inp.heartbeat_at is None:
            if inp.now - self.t_accept > ACCEPT_TIMEOUT_S:
                return 'heartbeat_timeout'
        elif inp.now - inp.heartbeat_at > HEARTBEAT_EXPIRY_S:
            return 'heartbeat_timeout'
        return None

    def _ingest(self, odom):
        """Update progress from a new odometry sample (once per distinct measurement)."""
        self.measured_v, self.measured_w = odom.v, odom.w
        if self._origin is None:
            return
        x0, y0, yaw0 = self._origin
        px, py, pyaw = self._prev
        self.path += math.hypot(odom.x - px, odom.y - py) if self.axis == 'linear' else abs(wrap(odom.yaw - pyaw))
        dx, dy = odom.x - x0, odom.y - y0
        if self.axis == 'linear':
            c, s = math.cos(yaw0), math.sin(yaw0)
            self.progress = dx * c + dy * s
            self.lateral = -dx * s + dy * c
            self.heading = wrap(odom.yaw - yaw0)
        else:
            self.progress += wrap(odom.yaw - pyaw)     # unwrapped incrementally across +-pi
            self.lateral = math.hypot(dx, dy)
            self.heading = (self.target - self.progress) if not self.is_step else 0.0
        self._prev = (odom.x, odom.y, odom.yaw)

    def _step(self, inp):
        now, odom = inp.now, inp.odom
        if odom is not None and odom.stamp_ns != self._last_stamp:
            self._last_stamp = odom.stamp_ns
            if self._prev is None:
                self._prev = (odom.x, odom.y, odom.yaw)
            self._ingest(odom)
        reason = self._abort_reason(inp)
        if reason:
            self._finish(ABORTED, reason)
            return self.command

        if self.phase == ACCEPTED:
            self._accepted(inp)
        if self.phase in (RUNNING, SETTLING):
            if self._travel_exceeded():
                self._finish(FAILED, 'travel_limit')
                return self.command
            if self.is_step:
                self._step_phase(now)
            else:
                self._position_phase(now)
        return self.command

    def _travel_exceeded(self):
        if self.is_step:
            limit = 2.0 if self.axis == 'linear' else TWO_PI
        else:
            limit = abs(self.target) + PATH_MARGIN[self.axis]
        return self.path > limit or (self.is_step and abs(self.progress) > limit)

    def _accepted(self, inp):
        """Hold zero until a post-acceptance heartbeat and a stationary baseline both exist."""
        now, odom = inp.now, inp.odom
        self.cmd = 0.0
        still = abs(odom.v) <= BASELINE_STILL_LINEAR and abs(odom.w) <= BASELINE_STILL_ANGULAR
        if not still:
            self._baseline_since = None
        elif self._baseline_since is None:
            self._baseline_since = now
        if inp.heartbeat_at is None or self._baseline_since is None:
            if now - self.t_accept > ACCEPT_TIMEOUT_S:
                self._finish(FAILED, 'not_stationary')
            return
        if now - self._baseline_since >= BASELINE_S:
            self._origin = self._prev = (odom.x, odom.y, odom.yaw)
            self.progress = self.lateral = self.heading = self.path = 0.0
            if not self.is_step and self.axis == 'rotation':
                self.heading = self.target
            self.phase = RUNNING
        elif now - self.t_accept > ACCEPT_TIMEOUT_S:
            self._finish(FAILED, 'not_stationary')

    def _ramp_to(self, desired):
        accel, decel = RAMP[self.axis]
        # decelerating = moving toward zero (or reversing)
        rate = decel if abs(desired) < abs(self.cmd) or desired * self.cmd < 0 else accel
        delta = max(-rate * DT, min(rate * DT, desired - self.cmd))
        self.cmd += delta

    def _position_phase(self, now):
        tol = LINEAR_TOL if self.axis == 'linear' else ANGULAR_TOL
        still = abs(self.measured_v if self.axis == 'linear' else self.measured_w) <= \
            (STILL_LINEAR if self.axis == 'linear' else STILL_ANGULAR)
        remaining = self.target - self.progress
        if abs(remaining) <= tol:
            self._ramp_to(0.0)
            if self.phase == RUNNING:
                self.phase = SETTLING
                self._still_since = None
            if still and self.cmd == 0.0:
                if self._still_since is None:
                    self._still_since = now
                if now - self._still_since >= SETTLE_S:
                    self._finish(COMPLETED, 'target_reached')
            else:
                self._still_since = None
            return
        self.phase, self._still_since = RUNNING, None
        slow = math.sqrt(2.0 * PLAN_DECEL[self.axis] * (abs(remaining) - tol))
        speed = min(self.speed, max(CREEP[self.axis], slow))
        self._ramp_to(math.copysign(speed, remaining))

    def _step_phase(self, now):
        if self.phase == RUNNING:
            if self._plateau_start is None:
                self._plateau_start = now
            if now - self._plateau_start < self.duration:
                self.cmd = self.target          # constant command; the drive's own limits shape the response
                return
            self.phase, self.cmd, self._zero_since = SETTLING, 0.0, now
        self.cmd = 0.0
        still = abs(self.measured_v) <= STILL_LINEAR and abs(self.measured_w) <= STILL_ANGULAR
        if not still:
            self._still_since = None
        elif self._still_since is None:
            self._still_since = now
        if self._still_since is not None and now - self._still_since >= SETTLE_S \
                and now - self._zero_since >= SETTLE_S:
            self._finish(COMPLETED, 'step_complete')
