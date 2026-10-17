import math

import pytest

from uet_amr_evaluation.motion import core
from uet_amr_evaluation.motion.core import DT, Inputs, MotionTestCore, Odom, wrap


class FakeRobot:
    """First-order drive with acceleration limits; yaw is kept unwrapped internally and reported wrapped."""

    def __init__(self, yaw=0.0, x=0.0, y=0.0):
        self.x, self.y, self.yaw, self.v, self.w = x, y, yaw, 0.0, 0.0
        self.stamp = 1

    def advance(self, cmd_v, cmd_w):
        self.v += max(-0.8 * DT, min(1.2 * DT, cmd_v - self.v))
        self.w += max(-1.2 * DT, min(1.8 * DT, cmd_w - self.w))
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT
        self.yaw += self.w * DT
        self.stamp += 1

    def odom(self):
        return Odom(self.stamp, self.x, self.y, wrap(self.yaw), self.v, self.w)


def run(core_, robot, seconds=80.0, hb=True, fault_at=None, on_tick=None, **overrides):
    """Drive the core at 20 Hz; returns list of (t, cmd_v, cmd_w, phase)."""
    log, t = [], 100.0
    hb_at = None
    for i in range(int(seconds / DT)):
        t = 100.0 + i * DT
        if hb and (hb is True or t < hb):
            hb_at = t
        inp = dict(now=t, odom=robot.odom(), odom_age=0.0, heartbeat_at=hb_at)
        if fault_at is not None and t >= fault_at:
            inp['hw_fault'] = True
        inp.update(overrides)
        if on_tick:
            on_tick(i, t, inp)
        v, w = core_.step(Inputs(**inp))
        log.append((t, v, w, core_.phase))
        robot.advance(v, w)
        if core_.terminal:
            break
    return log


def make(kind, target, speed=0.0):
    err, eff = core.validate_motion(kind, target, speed)
    assert err is None, err
    return MotionTestCore('r1', kind, target, eff, 100.0)


@pytest.mark.parametrize('target', [1.0, -0.6])
def test_linear_reaches_target_either_direction(target):
    robot, c = FakeRobot(yaw=0.7), make('linear', target)
    log = run(c, robot)
    assert c.phase == 'completed'
    assert abs(c.progress - target) <= core.LINEAR_TOL
    assert abs(c.lateral) < 0.01
    assert max(abs(v) for _, v, _, _ in log) <= 0.1 + 1e-9


def test_linear_projection_onto_initial_heading_reverse():
    robot, c = FakeRobot(yaw=math.pi), make('linear', 0.5)    # facing -x; forward progress = -dx
    run(c, robot)
    assert c.phase == 'completed' and robot.x < -0.45


@pytest.mark.parametrize('target,yaw0', [(math.radians(90), 0.0), (-math.radians(200), 2.0),
                                          (2 * math.pi, 0.5), (math.radians(170), 3.0)])
def test_rotation_unwraps_across_pi(target, yaw0):
    robot, c = FakeRobot(yaw=yaw0), make('rotation', target)
    run(c, robot)
    assert c.phase == 'completed'
    assert abs(c.progress - target) <= core.ANGULAR_TOL
    assert abs(robot.yaw - yaw0 - target) < 0.05      # truly turned the full amount, not the wrapped one


def test_overshoot_is_corrected_within_tolerance():
    robot, c = FakeRobot(), make('linear', 0.5)
    state = {}

    def kick(i, t, inp):                       # shove the robot past the target mid-run
        if c.phase == 'running' and c.progress > 0.3 and 'done' not in state:
            robot.x += 0.12
            state['done'] = True
    run(c, robot, on_tick=kick)
    assert c.phase == 'completed' and abs(c.progress - 0.5) <= core.LINEAR_TOL


def test_no_motion_before_first_post_acceptance_heartbeat():
    robot, c = FakeRobot(), make('linear', 0.5)
    log = run(c, robot, seconds=4.0, hb=False)
    assert all(v == 0 and w == 0 for _, v, w, _ in log) and c.phase == 'accepted'
    log = run(make('linear', 0.5), FakeRobot(), seconds=6.0, hb=False)
    assert log[-1][3] == 'aborted'                  # never saw a heartbeat -> abort, still no motion
    assert all(v == 0 for _, v, _, _ in log)


def test_baseline_of_half_second_before_motion():
    robot, c = FakeRobot(), make('linear', 0.5)
    log = run(c, robot)
    first = next(t for t, v, _, _ in log if v != 0)
    assert first - 100.0 >= core.BASELINE_S - 1e-9


@pytest.mark.parametrize('stop_after', [0.2, 3.0])      # baseline and mid-motion
def test_heartbeat_loss(stop_after):
    robot, c = FakeRobot(), make('linear', 1.0)
    log = run(c, robot, hb=100.0 + stop_after)
    assert c.phase == 'aborted' and c.reason == 'heartbeat_timeout'
    assert log[-1][1] == 0.0


def test_stale_odometry_aborts_with_zero_command():
    robot, c = FakeRobot(), make('linear', 1.0)

    def stale(i, t, inp):
        if t > 103.0:
            inp['odom_age'] = 0.3
    log = run(c, robot, on_tick=stale)
    assert (c.phase, c.reason) == ('aborted', 'stale_odometry') and log[-1][1] == 0.0


@pytest.mark.parametrize('override,reason', [
    (dict(hw_fault=True), 'hardware_fault'), (dict(controller_active=False), 'controller_inactive'),
    (dict(ownership_ok=False), 'ownership_lost'), (dict(mode_ok=False), 'mode_lost'),
    (dict(stop_requested=True), 'stop_requested')])
def test_abort_conditions(override, reason):
    robot, c = FakeRobot(), make('linear', 1.0)
    run(c, robot, on_tick=lambda i, t, inp: inp.update(override) if t > 103.0 else None)
    assert (c.phase, c.reason) == ('aborted', reason) and c.command == (0.0, 0.0)


def test_timeout():
    c2, r2 = make('linear', 2.0), FakeRobot()
    r2.advance = lambda *_: setattr(r2, 'stamp', r2.stamp + 1)      # robot that never moves
    run(c2, r2, seconds=70.0)
    assert (c2.phase, c2.reason) == ('aborted', 'timeout')


def test_travel_limit_when_robot_runs_away():
    robot, c = FakeRobot(), make('linear', 0.3)
    orig = robot.advance
    robot.advance = lambda v, w: orig(0.2 if c.phase in ('running', 'settling') else v, w)   # ignores the command
    run(c, robot)
    assert (c.phase, c.reason) == ('failed', 'travel_limit')


def test_terminal_state_is_sticky():
    robot, c = FakeRobot(), make('linear', 0.1)
    run(c, robot)
    assert c.phase == 'completed'
    assert c.step(Inputs(now=500.0, odom=robot.odom(), heartbeat_at=500.0, stop_requested=True)) == (0.0, 0.0)
    assert c.phase == 'completed'


def test_internal_error_yields_zero():
    c = make('linear', 0.5)
    assert c.step(Inputs(now=100.0, odom=object(), heartbeat_at=100.0)) == (0.0, 0.0)
    assert c.phase == 'failed'


@pytest.mark.parametrize('args', [('linear', math.nan, 0.1), ('linear', 0.01, 0.1), ('linear', 2.5, 0.1),
                                  ('linear', 1.0, 0.3), ('rotation', 0.01, 0.3), ('rotation', 7.0, 0.3),
                                  ('rotation', 1.0, 0.7), ('spin', 1.0, 0.1), ('linear', 1.0, -0.1),
                                  ('linear', 1.0, math.inf)])
def test_motion_validation_rejects(args):
    assert core.validate_motion(*args)[0]


def test_motion_validation_defaults():
    assert core.validate_motion('linear', -1.0, 0.0) == (None, 0.1)
    assert core.validate_motion('rotation', 2 * math.pi, 0.0) == (None, 0.3)
    assert core.validate_motion('rotation', math.radians(5), 0.6)[0] is None


@pytest.mark.parametrize('args', [('linear', 0.0, 2), ('linear', 0.21, 2), ('linear', 0.2, 10),
                                  ('angular', 0.6, 10), ('angular', 0.1, 0.5), ('angular', 0.1, 11),
                                  ('linear', math.nan, 2), ('sideways', 0.1, 2), ('linear', 0.1, math.inf)])
def test_step_validation_rejects(args):
    assert core.validate_step(*args)


def test_step_validation_accepts_edges():
    assert core.validate_step('linear', 0.2, 9.0) is None
    assert core.validate_step('angular', -0.6, 1.0) is None
    assert core.validate_step('angular', 0.5, 10.0) is None     # 5.0 rad <= 2*pi - 0.5


@pytest.mark.parametrize('axis,vel', [('linear', 0.15), ('linear', -0.15), ('angular', 0.5), ('angular', -0.5)])
def test_velocity_step_profile(axis, vel):
    robot, c = FakeRobot(), MotionTestCore.for_step('s1', axis, vel, 3.0, 100.0)
    log = run(c, robot)
    assert c.phase == 'completed' and c.kind == f'{axis}_velocity'
    nonzero = [(t, v, w) for t, v, w, _ in log if (v if axis == 'linear' else w) != 0]
    start = nonzero[0][0]
    assert start - 100.0 >= core.BASELINE_S - 1e-9                        # baseline first
    cmds = {round(v if axis == 'linear' else w, 9) for _, v, w in nonzero}
    assert cmds == {round(vel, 9)}                                        # constant, un-ramped step
    assert abs(nonzero[-1][0] - start + DT - 3.0) < 1e-6
    assert log[-1][0] - nonzero[-1][0] >= core.SETTLE_S                   # braking + >=0.5 s settling recorded
    assert abs(c.measured_v) < 0.01 and (c.progress > 0) == (vel > 0)


def test_step_plateau_starts_at_first_nonzero_command():
    robot, c = FakeRobot(), MotionTestCore.for_step('s1', 'linear', 0.1, 2.0, 100.0)
    log = run(c, robot)
    ticks = sum(1 for _, v, _, _ in log if v != 0)
    assert ticks == 40


def test_step_hard_travel_limit_enforced_independently():
    robot, c = FakeRobot(), MotionTestCore.for_step('s1', 'linear', 0.1, 5.0, 100.0)
    robot.advance = lambda v, w, o=robot.advance: o(0.2 if c.phase in ('running', 'settling') else v, w)   # runaway plant
    run(c, robot)
    assert (c.phase, c.reason) == ('failed', 'travel_limit')


def test_repeated_measurement_stamp_does_not_double_count():
    robot, c = FakeRobot(), make('linear', 0.5)
    calls = []

    def freeze(i, t, inp):       # odometry repeats every other tick
        if i % 2:
            inp['odom'] = calls[-1]
        calls.append(inp['odom'])
    run(c, robot, on_tick=freeze)
    assert c.phase == 'completed' and abs(c.progress - 0.5) <= core.LINEAR_TOL
