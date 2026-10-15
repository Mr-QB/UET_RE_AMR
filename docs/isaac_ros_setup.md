# Isaac ROS + Docker Dev Setup

GPU-accelerated 3D reconstruction (`nvblox_ros`) feeding Nav2's local costmap
(`nvblox_nav2`), built and run inside NVIDIA's Isaac ROS dev container.

Separate from the native setup in [`getting_started.md`](getting_started.md).
Use this when you need `nvblox_ros`; use the native flow otherwise.

---

## Prerequisites

- **x86_64**: NVIDIA GPU, [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), Docker, 32+ GB free disk.
- **Jetson**: JetPack, Docker, a 128+ GB NVMe SSD (the stock eMMC/SD card isn't enough once the base images are pulled).
- `git-lfs` installed (`apt install git-lfs`) — the Isaac ROS repos ship
  prebuilt binaries (e.g. `isaac_ros_gxf`'s `libgxf_core.so`) via Git LFS.
  `scripts/setup.sh docker-dev` runs `git lfs pull` for you, but only if the
  `git-lfs` binary exists; without it you'll get a cryptic linker "syntax
  error" at build time instead of the real binary.
- Sanity check either way: `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`.

No GPU? Everything still builds — `uet_amr_navigation` just falls back to the
plain CPU costmap it always had. See [No-GPU fallback](#no-gpu-fallback).

---

## Quick start

```bash
git clone https://github.com/UET-RE/UET_RE_AMR.git
cd UET_RE_AMR
./scripts/setup.sh docker-dev
```

This fetches `isaac_ros_common` into `ros2/third_party/` (pinned in
`ros2/third_party.lock`; `nvblox`/`nitros` are *not* vendored — they come from
NVIDIA's apt repo inside the container), wires up the serial-device override, and
hands off to `isaac_ros_common`'s own `run_dev.sh`, which builds the image and drops
you into the container with `ros2/` mounted at `/workspaces/isaac_ros-dev`.

Inside the container:

```bash
rosdep install --from-paths src third_party --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch uet_amr_navigation navigation.launch.py use_nvblox:=true
```

No `--packages-skip` needed — `nvblox_ros`, `nvblox_nav2` and `nitros` resolve to
NVIDIA's apt binaries through `rosdep`.

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
- `isaac_ros_common` (only its scripts/Dockerfiles are used) and
  `rplidar_ros` (bare metal) and `librealsense` (Jetson bare metal) are fetched into `ros2/third_party/` by
  `scripts/fetch_third_party.sh`, pinned in `ros2/third_party.lock`.

---

## No-GPU / native builds

The native flow (`scripts/setup.sh native-dev` / `native-prod`) has no Isaac ROS sources
and no NVIDIA apt repo, so `nvblox_ros`/`nvblox_nav2` aren't installed there and
`navigation.launch.py` uses the plain CPU costmap (`use_nvblox` defaults to off
without them). Use the container or the production image to run nvblox.

---

## Troubleshooting

**Linker "syntax error" on a `.so` file** (e.g.
`libgxf_core.so:1: syntax error`) — that file is a Git LFS pointer, not the
real binary (`git-lfs` wasn't installed, or `git lfs pull` never ran for that
repo). Fix: `cd ros2/third_party/<repo> && git lfs install --local
&& git lfs pull`, then rebuild. `scripts/setup.sh docker-dev` does this for you,
but only if `git-lfs` is installed on your machine.

**`run_dev.sh` not found** — the vendored layout may have moved past
`release-3.2`. Check NVIDIA's current docs against the pin in `ros2/third_party.lock`.

**`nvidia-smi` fails in the container** — check `nvidia-container-toolkit` is
installed and `docker info` lists `nvidia` as a runtime. On Jetson, check
JetPack + `nvidia-docker2`.

**Costmap plugin not found (`nvblox::nav2::NvbloxCostmapLayer`)** — you
launched with `use_nvblox:=true` but `nvblox_nav2` isn't installed. Run the
`colcon build` above (or install `ros-humble-nvblox-nav2`), or launch with
`use_nvblox:=false`.

**librealsense2 / realsense-ros** — unrelated to this doc, see
[`jetson_realsense_setup.md`](jetson_realsense_setup.md) and
[`d435i_guide.md`](d435i_guide.md).

---

## Development loop (Jetson and x86)

`docker-dev` builds one image from NVIDIA's `isaac_ros_common` layers and gives you two ways to use
it. The image is NVIDIA's *prebuilt* base (pulled when one is published for the pinned revision),
plus on a Jetson NVIDIA's `realsense` layer (librealsense 2.55.1 RSUSB + `realsense-ros` 4.51.1-isaac,
built once), plus `docker/Dockerfile.uet_amr`, which bakes in the workspace's apt dependencies
(nvblox, nitros, nav2, rplidar_ros, ...). `ros2/` is mounted at `/workspaces/isaac_ros-dev`, so
`build/ install/ log/` live on the host and builds are incremental.

```bash
./scripts/setup.sh docker-dev                # interactive shell via NVIDIA's run_dev.sh
```

For a fast "pull and rebuild" loop, use the long-lived container `uet_amr_dev` (it survives exits
and restarts with the machine, so nothing is reinstalled):

```bash
git pull
./scripts/setup.sh docker-dev build          # image rebuilds only if a package.xml changed, then colcon
./scripts/setup.sh docker-dev run ros2 launch uet_amr_bringup amr_bringup.launch.py
./scripts/setup.sh docker-dev shell
./scripts/setup.sh docker-dev down           # remove the container; build/ install/ stay on disk
docker compose -f docker/compose.zenoh.yml up -d   # optional Zenoh bridge for a remote PC (docker/README.md)
```

Why it is fast:
- **Dependencies are in the image,** not reinstalled by `rosdep` on every new container.
- **`--symlink-install`:** edits to launch files, configs and Python need no rebuild at all.
- **Long-lived container:** `run_dev.sh` uses `docker run -it --rm`, which discards the container
  (and whatever you installed in it) on exit.

Prebuilt vs. built from scratch: NVIDIA's registry has prebuilt x86_64 images only up to
`isaac_ros_common` `a40a017`. Jetson uses the newer `fcf4d9e` (it pins TensorRT 10.3 to match
JetPack), so `ros2/third_party.lock` has an `isaac_ros_common@x86_64` override. If you move either
pin and no prebuilt matches, `run_dev.sh` builds the whole stack from source, which takes hours.

Needs: Docker with the NVIDIA runtime, `git-lfs`. The first run downloads and unpacks the base
(40–60 GB), so allow plenty of disk and a long first start. Verified on x86 (the workspace builds in
the container); not yet run on a Jetson.
