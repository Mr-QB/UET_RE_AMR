# Intel RealSense D435i

The D435i supplies depth + colour to nvblox and the IMU to the EKF. It must be on a **USB 3.0+ port**. Point it at
textured, well-lit areas when mapping; blank walls and mirrors give poor depth.

Installation is part of the image (see [setup.md](setup.md)): apt packages on x86, a source-built librealsense 2.58.4 / realsense-ros 4.58.4
on Jetson (the kernel V4L2 backend does not work on L4T).

## What the robot runs

`uet_amr_bringup/launch/depth_camera.launch.py` wraps `realsense2_camera` with:

- depth and colour at 640x480 @ 30 Hz, `enable_sync` (without it colour lags depth by about 60 ms),
- gyro + accel united into one IMU stream (`unite_imu_method: 2`) on `/camera/camera/imu`, bias-corrected by
  `uet_amr_localization/imu_bias_corrector` before the EKF,
- `align_depth` on, so `/camera/camera/aligned_depth_to_color/*` matches the colour image (needed by people segmentation).

## Checks

```bash
scripts/amr run rs-enumerate-devices                          # camera listed?
scripts/amr run ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw   # stable 15-30 Hz?
```

A low rate (about 5 Hz) usually means the UDP buffer limits are not applied (run `scripts/amr setup`).

## IMU calibration

For accurate mapping, calibrate the internal IMU with Intel's
[`rs-imu-calibration.py`](https://github.com/IntelRealSense/librealsense/tree/master/tools/rs-imu-calibration).
