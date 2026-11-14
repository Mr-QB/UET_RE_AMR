# Tuning log

Field-test results and the history of tuned values. Keep YAML comments to *why a value is what it is*; put "was X" and measurements here.
Add newest entries on top. Commit tuning-only changes as `tune(<scope>): <what and why>`.

## Carried over from YAML comments and git history (to 2026-11-14)

- **Wheel odometry** (`uet_amr_bringup/config/controllers.yaml`): effective `wheel_separation` has been 0.304 (URDF), 0.45, 0.5365, 1.0 and now
  0.993 with left/right radius multipliers 1.0282 / 1.0298. It is a calibrated effective value, not geometry.
- **Odometry/IMU trust**: the wheel yaw-rate covariance went 0.01 -> 0.25 -> 0.01 on 2026-11-06 (a bias-corrected IMU was supposed
  to be trusted over a wheel spinning in the air; reverted). `odom0_twist_rejection_threshold: 5.0` in
  `uet_amr_localization/config/ekf.yaml` was added after wheel-odom twist spikes of about 4.2 m/s (limit 0.7) in a field test.
- **Speed**: `linear.x.max_velocity` 0.5 -> 0.4 -> 1.0 -> 0.7 m/s. Nav2 `desired_linear_vel` follows it (0.7).
- **Inflation radius** (Nav2 costmaps): 0.30 -> 0.55 -> 0.45 -> 0.4; now global 0.55 / local 0.4.
- **Goal tolerance**: xy/yaw 0.1/0.1 -> 0.15/0.25 -> 0.10/3.14 (yaw effectively ignored).
- **Footprint**: round 0.30 -> rectangle; the local costmap uses a half-width of 0.15 so the robot fits through doors.
  `nvblox_footprint_filter` pads the footprint to 0.34 (front/rear) and 0.31 (half-width).
- **Planner**: NavFn -> SmacPlanner2D (a footprint-aware SmacPlannerHybrid trial was dropped; see git history of `nav2.yaml`).
- **allow_reversing**: false -> true -> false.
- **Firmware-driven compensations** (see `docs/getting_started.md`): command scales 100 -> 9.5493 -> 25 -> split 18.5 linear / 39.5 angular.
