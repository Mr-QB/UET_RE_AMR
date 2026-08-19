# RealSense on Jetson

## Why not `ros-humble-realsense2-camera`?

The apt package `ros-humble-realsense2-camera` depends on apt's `librealsense2`,
which is a generic prebuilt binary. It doesn't work correctly on Jetson (L4T /
aarch64): it lacks the RSUSB backend and the kernel patches Jetson needs, so the
camera fails to enumerate or stream frames.

`tools/setup_prod.sh` detects Jetson automatically (via `/etc/nv_tegra_release`)
and skips this apt package — see `is_jetson()` / `install_ros2_deps()` in
`tools/common.sh`.

## What you need to do on the Jetson

1. **Build and install `librealsense2` from source yourself**, following Intel's
   official Jetson instructions:
   https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

   The key point is building with `-DFORCE_RSUSB_BACKEND=true`, which avoids
   needing to patch/rebuild the L4T kernel.

2. **Run `tools/setup_prod.sh`.** Once it detects Jetson, it pulls in
   `realsense-ros` (the ROS 2 wrapper) as workspace source —
   `ros2/src/third_party/realsense-ros` — instead of relying on the apt ROS
   package. `colcon build` then compiles it against the `librealsense2` you
   installed in step 1, and `rosdep install` is told to skip the `librealsense2`
   key (it's already satisfied by your manual install, not apt).

## Verify

```bash
# Confirm librealsense2 sees the camera
rs-enumerate-devices

# Confirm the ROS2 wrapper streams
ros2 launch realsense2_camera rs_launch.py
```

If `rs-enumerate-devices` doesn't find the camera, the librealsense2 install
(step 1) needs attention before touching the ROS side.
