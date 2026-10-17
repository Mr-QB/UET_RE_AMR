# uet_amr_evaluation

Odometry evaluation tools and the robot side of the web dashboard's **motion tests**.

| Part | What it is |
|---|---|
| `odom_test`, `odom_report` | Interactive CLI odometry tests (straight / rotation) with CSV + plots. Drives the controller directly; use only with nothing else running. |
| `uet_amr_evaluation/motion/` | Motion-test executor, velocity-step executor and the local CDR recorder used by the web dashboard. |
| `export_motion_archive` | Exports a finished run as the dashboard's robot-archive JSON. |

## Motion-test interfaces (`uet_amr_msgs`)

| Name | Type | Notes |
|---|---|---|
| `/amr/start_motion_test` | `StartMotionTest.srv` | `linear` (m) / `rotation` (rad), signed target. `speed_limit` 0 selects the default (0.1 m/s, 0.3 rad/s); above 0.2 m/s / 0.6 rad/s is rejected. |
| `/amr/start_velocity_step` | `StartVelocityStep.srv` | Constant-velocity step, 1–10 s, plateau travel ≤ 1.8 m / 2π−0.5 rad. |
| `/amr/stop_motion_test` | `StopMotionTest.srv` | Idempotent; only affects the matching run. |
| `/amr/motion_test/state` | `MotionTestState.msg` | 20 Hz, terminal state repeats until the next start. Idle readiness is published too. |
| `/amr/motion_test/heartbeat` | `std_msgs/String` | Exact run ID every 100 ms. Expires after 500 ms (steady clock). |

Run IDs are single-use for the process lifetime (rejected ones too) and must match `[A-Za-z0-9][A-Za-z0-9_.-]{0,127}`.
A Start that times out on the client must not be retried; read `/amr/motion_test/state` instead.

## How it is wired

* The executor is created inside `amr_mode_manager` and **holds the mode manager's operation lock for the whole run**,
  so `/amr/set_mode` and `/amr/save_map` are refused while a test runs. It never changes modes or cancels navigation.
* It drives through `teleop_gate`'s private `test` source (`/teleop/test/cmd_vel_raw`, acquired through
  `/amr/internal/test_source`). `/amr/set_teleop` can not select `test` and is refused while it is held. The gate clamps
  test commands to 0.2 m/s / 0.6 rad/s and drops the source (zero command, back to `none`) if the executor goes quiet
  for 0.5 s.
* Start requires: Mapping (`slam`) mode, manual input Off, active `diff_drive_controller`, fresh odometry (< 250 ms,
  frame `odom`), a recent non-fault `/hardware/status`, no other node publishing to the controller input, and a free
  operation lock.
* Abort reasons: `stop_requested`, `mode_lost`, `ownership_lost`, `controller_inactive`, `hardware_fault`,
  `stale_odometry`, `heartbeat_timeout`, `timeout` (phase `aborted`); `travel_limit`, `not_stationary`,
  `internal_error` (phase `failed`). Every exit publishes zero, releases the test source and the lock.
* No nonzero command is sent before the first heartbeat received after acceptance **and** 0.5 s of stationary
  baseline at 20 Hz. Velocity steps then hold the commanded velocity unramped for `duration_s` (the drive controller's
  acceleration limits shape the response) and record braking plus ≥ 0.5 s of stationary settling.
* Position tests ramp the command, decelerate with a square-root profile, and settle for 0.5 s inside 0.02 m / 2°.
* `heading_error`: `yaw − initial yaw` for linear runs, `target − progress` for rotations.
  `lateral_error`: perpendicular offset (linear) or distance from the start position (rotation).

## Local recording and archive export

From acceptance until 0.5 s after the terminal state the executor stores native-rate CDR for `/odometry/filtered`,
`/diff_drive_controller/odom`, `/hardware/status` and `/amr/motion_test/state` in `~/amr_motion_tests/`
(`motion_test.archive_dir` parameter), with wall and steady receipt times. Capture goes through a bounded queue to a
disk worker, so it never delays stopping. A queue drop, disk error or the 20,000-packet / 16 MiB limit marks the run
`incomplete`; an interrupted run stays `capturing`; neither is ever exported as complete.

```bash
ros2 run uet_amr_evaluation export_motion_archive <run_id> [--dir ~/amr_motion_tests] [--out file.json]
```

Upload the JSON through the dashboard run inspector or `POST /api/experiments/<run_id>/robot-archive`. Re-exporting gives
a byte-identical archive (stable `archive_id`), so re-uploads are idempotent. `<run_id>.params.json` next to it holds the
parameter snapshot (controller acceleration limits, EKF settings, executor ramps).

## Tests

```bash
cd ros2/src/uet_amr_evaluation && python3 -m pytest test        # needs a sourced ROS 2 + uet_amr_msgs for the integration test
```

`test_motion_core.py` (fake odometry: reverse projection, ±π crossing, full rotation, overshoot, stale odometry,
heartbeat loss, faults, travel limit, steps on both axes), `test_motion_recorder.py` (archive round trip, drops, limits,
interruption) and `test_motion_integration.py` (executor + real teleop gate over DDS with a fake plant).
