# Isaac ROS + Docker Dev Setup

GPU-accelerated 3D reconstruction (`nvblox_ros`) feeding Nav2's local costmap
(`nvblox_nav2`), built and run inside NVIDIA's Isaac ROS dev container.

Separate from the native setup in [`getting_started.md`](getting_started.md).
Use this when you need `nvblox_ros`; use the native flow otherwise.

---

## Prerequisites

- **x86_64**: NVIDIA GPU, [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), Docker, 32+ GB free disk.
- **Jetson**: JetPack, Docker, a 128+ GB NVMe SSD (the stock eMMC/SD card isn't enough once the base images are pulled).
- Sanity check either way: `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`.

No GPU? Everything still builds — `uet_amr_navigation` just falls back to the
plain CPU costmap it always had. See [No-GPU fallback](#no-gpu-fallback).

---

## Quick start

```bash
git clone --recurse-submodules https://github.com/UET-RE/UET_RE_AMR.git
cd UET_RE_AMR
./tools/setup_isaac_ros.sh
```

This pulls the `isaac_ros_common`/`isaac_ros_nitros`/`isaac_ros_nvblox`
submodules, wires up the serial-device override, and hands off to
`isaac_ros_common`'s own `run_dev.sh`, which builds the image and drops you
into the container with `ros2/` mounted at `/workspaces/isaac_ros-dev`.

Inside the container:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-skip isaac_ros_nvblox nvblox_examples_bringup \
  --packages-skip-by-dep isaac_ros_nvblox nvblox_examples_bringup
source install/setup.bash
ros2 launch uet_amr_navigation navigation.launch.py use_nvblox:=true
```

(That `--packages-skip` pair is permanent, not a GPU thing — those two
packages need NVIDIA's people-detection model downloads, which we don't
vendor and don't need.)

Start the micro-ROS agent separately, on the host — the container doesn't
run it for you:

```bash
docker run --rm --network host --device=/dev/ttyUSB0 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0 --baudrate 115200
```

**VS Code:** `.devcontainer/devcontainer.json` opens the same container as
the right non-root user. Build the image with the script above first — VS
Code doesn't build it for you. On Jetson, change `"image"` to
`isaac_ros_dev-aarch64`.

---

## What's what

- **`nvblox_ros`** turns the D435i depth stream into a 3D reconstruction,
  using the existing wheel+IMU EKF for pose (no VSLAM — see
  [ADR 0003](adr/0003-nvblox-pose-from-ekf-not-vslam.md)).
- **`nvblox_nav2`** is the Nav2 costmap plugin that reads nvblox's output.
  `nav2_nvblox.yaml` loads it instead of `nav2.yaml`'s plain `voxel_layer`.
- **`navigation.launch.py`**'s `use_nvblox` arg picks between them — defaults
  to on if it detects an NVIDIA GPU, off otherwise. Override either way:
  `ros2 launch uet_amr_navigation navigation.launch.py use_nvblox:=true`.
- `uet_amr_navigation` only *exec*-depends on the nvblox packages, not
  *build*-depends — the plugin loads by name at runtime, so this package
  builds fine with or without a GPU.
- `isaac_ros_nitros` is a separate NVIDIA repo that `nvblox_ros` needs for
  its message transport — nothing here calls it directly, it just has to be
  vendored alongside `isaac_ros_nvblox`.
- All three Isaac ROS repos live under `ros2/src/third_party/` as git
  submodules, same as `rplidar_ros`/`realsense-ros`, pinned to `release-3.2`.

---

## No-GPU fallback

`tools/common.sh`'s `has_nvidia_gpu` gates the build. Without a GPU,
`build_workspace()` skips `nvblox_ros`, `isaac_ros_gxf`, and
`isaac_ros_managed_nitros` (plus everything depending on them). `nvblox_nav2`
and `nvblox_msgs` have no CUDA dependency and always build, so the costmap
plugin is available even when nvblox itself isn't running.

Not yet verified against a real build — no GPU/Docker toolchain in dev. The
skip lists in `tools/common.sh` may need small fixes on the first real run.

---

## Troubleshooting

**`run_dev.sh` not found** — the vendored layout may have moved past
`release-3.2`. Check NVIDIA's current docs against the pin in `.gitmodules`.

**`nvidia-smi` fails in the container** — check `nvidia-container-toolkit` is
installed and `docker info` lists `nvidia` as a runtime. On Jetson, check
JetPack + `nvidia-docker2`.

**Costmap plugin not found (`nvblox::nav2::NvbloxCostmapLayer`)** — you
launched with `use_nvblox:=true` but never built `nvblox_nav2`. Run the
`colcon build` above, or launch with `use_nvblox:=false`.

**librealsense2 / realsense-ros** — unrelated to this doc, see
[`jetson_realsense_setup.md`](jetson_realsense_setup.md) and
[`d435i_guide.md`](d435i_guide.md).
