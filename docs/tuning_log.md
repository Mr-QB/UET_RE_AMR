# Tuning log

Field-test results and the history of tuned values. Keep YAML comments to *why a value is what it is*; put "was X" and measurements here.
Add newest entries on top. Commit tuning-only changes as `tune(<scope>): <what and why>`.

## 2026-11-08 field test

Findings and what happened to them:

- Wheel-odometry heading drifts 0.22 deg/m (linear in distance). Suspected left/right radius mismatch of about 0.16 %. Radius multipliers were
  changed to 1.0282 / 1.0298 in `88850be`; **the 20 m straight-line confirmation is still open.**
- Two wheel-odom samples at 4.2 m/s (limit 0.7) were accepted by the EKF: `odom0_twist_rejection_threshold: 5.0` added in `88850be`.
- AMCL covariance jumped to metres at rest (random-particle injection): `recovery_alpha_fast/slow` set to 0 in `88850be`.
- 39 % of leg time went to turning to the goal heading: `yaw_goal_tolerance` relaxed to 3.14 in `88850be`.
- Not acted on yet: accelerometer reads 1.0 % high, camera 3.7 deg off level (fix before fusing the accelerometer); drive-board current signal
  is unusable (spikes to 20.4 A, does not follow load); GPU mean load 89 % leaves little headroom; gyro yaw offset moved from -2.55 to -2.24 mrad/s
  over 35 minutes, so the bias corrector must keep learning (a 10 minute rest would measure bias instability).

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
