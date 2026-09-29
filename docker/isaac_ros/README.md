# Isaac ROS Dev Container

Built and launched through `isaac_ros_common`'s own `scripts/run_dev.sh`
(vendored submodule, pinned to `release-3.2`), not a custom Dockerfile —
it already handles x86_64/Jetson base image selection and GPU wiring.

Don't run `run_dev.sh` directly — use `tools/setup_isaac_ros.sh`, which
inits the submodules, drops `config/isaac_ros_dev-dockerargs` into place
(adds the serial device; see that file for what not to duplicate), then
runs `run_dev.sh -d ros2`.

Full workflow: [`docs/isaac_ros_setup.md`](../../docs/isaac_ros_setup.md).

Micro-ROS agent isn't started automatically — run it separately:

```bash
docker run --rm --network host --device=/dev/ttyUSB0 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0 --baudrate 115200
```

If `.gitmodules` gets re-pinned past `release-3.2`, re-check `run_dev.sh` in
that release — NVIDIA has changed the dockerargs mechanism before.
