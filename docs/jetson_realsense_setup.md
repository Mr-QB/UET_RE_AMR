# RealSense on Jetson

## Why not just `ros-humble-realsense2-camera`?

The apt package depends on apt's `ros-humble-librealsense2`, which uses the kernel
V4L2 backend. Jetson (L4T) kernels aren't patched for that, so the camera fails to
enumerate or stream.

The fix is a `librealsense` built with `-DFORCE_RSUSB_BACKEND=ON` (userspace libusb,
no kernel patches). The apt `realsense2_camera` links the same soname
(`librealsense2.so.2.58`) as our build, so we keep the apt ROS wrapper and only
replace the library — **no `realsense-ros` source build is needed.**

## Bare metal (automatic)

`scripts/setup.sh native-prod` (or `native-dev`), on a Jetson (detected via
`/etc/nv_tegra_release`):

1. installs `ros-humble-realsense2-camera` from apt,
2. fetches librealsense (pinned in `ros2/third_party.lock`) to `ros2/third_party/librealsense`,
3. builds it once with RSUSB, no CUDA/examples/Python bindings,
4. installs it to `/usr/local` and copies it over the apt library in
   `/opt/ros/humble/lib/aarch64-linux-gnu/`, then `apt-mark hold ros-humble-librealsense2`
   so an `apt upgrade` doesn't undo it.

Re-running the script is safe; nothing is rebuilt once it's in place. The udev rule
(`hardware/udev/99-realsense-libusb.rules`) is installed by the same script.

## Docker

On a Jetson, `scripts/setup.sh docker-dev` stacks NVIDIA's own `realsense` layer onto the base
(`isaac_ros_common`'s `Dockerfile.realsense`): librealsense **v2.55.1** (RSUSB backend, CUDA) plus
their `realsense-ros` **4.51.1-isaac** fork. It is built once and cached. The robot host needs only
the udev rule (`hardware/udev/99-realsense-libusb.rules`, installed by `scripts/setup.sh`).

Note the version difference: bare metal runs librealsense 2.58.4 with the apt ROS wrapper 4.58.4;
the Docker environment runs 2.55.1 with the 4.51.1-isaac wrapper.

## Verify

```bash
rs-enumerate-devices                        # camera listed? (host, or: docker run --rm --privileged -v /dev:/dev uet_amr:prod rs-enumerate-devices)
ros2 launch realsense2_camera rs_launch.py  # frames streaming?
```

If `rs-enumerate-devices` doesn't find the camera, fix that before touching the ROS side.
