# Setup (Jetson robot and NVIDIA dev PC)

The software runs in one Docker container built on NVIDIA's [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS) layers.
Two targets are supported:

- **Robot: NVIDIA Jetson** (JetPack, Docker, a 128+ GB NVMe SSD; the stock eMMC/SD card is too small for the base images).
- **Dev PC: x86_64 with an NVIDIA GPU**, [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), Docker, 32+ GB free disk.
  Use it to develop the Isaac ROS parts (nvblox, perception) on your own machine; it runs the same image and the same commands.

Native installs, machines without an NVIDIA GPU and Gazebo simulation are not supported.
`git-lfs` is required on the host (`apt install git-lfs`): the Isaac ROS repos ship prebuilt binaries through it.
Sanity check: `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`.

## Quick start

```bash
git clone https://github.com/UET-RE/UET_RE_AMR.git && cd UET_RE_AMR
./scripts/amr setup            # udev rules, UDP buffer sysctl, isaac_ros_common, image (first run pulls 40-60 GB)
./scripts/amr build            # incremental colcon build inside the container
./scripts/amr run ros2 launch uet_amr_bringup amr_bringup.launch.py
```

On the robot add `--robot` to `setup` to install `uet-amr.service`, which starts bringup at boot
(`scripts/amr logs` follows it, `sudo systemctl start uet-amr` starts it now).

| Command | What it does |
|---|---|
| `scripts/amr setup [--robot]` | Host setup. Safe to re-run. |
| `scripts/amr build` | Updates the image only if `docker/` or a `package.xml` dependency changed, then `colcon build --symlink-install`. |
| `scripts/amr shell` / `run <cmd>` | Shell or one command in the long-lived container `uet_amr_dev` (`ros2/` is mounted at `/workspaces/isaac_ros-dev`, so `build/ install/ log/` stay on the host). |
| `scripts/amr down` | Remove the container (and the zenoh bridge). `build/ install/` stay. |
| `scripts/amr --rebuild build` | Force an image rebuild. |

Why it is fast: dependencies are baked into the image, `--symlink-install` means launch/config/Python edits need no rebuild,
and the container is long-lived. Don't call NVIDIA's `run_dev.sh` or `build_image_layers.sh` directly.

DDS: domain 42 and CycloneDDS, set once in `docker/amr.env`. Machines that talk to the robot need the same values
(see [docker/README.md](../docker/README.md) for the optional zenoh bridge to a remote PC).

## What's in the image

NVIDIA's prebuilt base (pulled when one is published for the pinned revision) + on a Jetson a RealSense layer +
`docker/Dockerfile.uet_amr` (the workspace's apt dependencies: nvblox, nitros, nav2, rplidar_ros, ...).

- **isaac_ros_common** is pinned in `ros2/third_party.lock`. NVIDIA's registry has prebuilt x86_64 images only up to
  `a40a017`; Jetson uses the newer `fcf4d9e` (TensorRT 10.3, matching JetPack), hence the `@x86_64` override.
  If you move a pin and no prebuilt matches, the whole stack builds from source, which takes hours.
- **RealSense on Jetson**: the apt `ros-humble-realsense2-camera` can't work on L4T kernels (V4L2 backend), so
  `Dockerfile.uet_amr_realsense` builds librealsense **2.58.4** (RSUSB userspace backend, CUDA) and realsense-ros **4.58.4**
  over NVIDIA's own 2.55.1 layer (2.55.1 spams IMU errors on D435i firmware 5.17.x). The ~15 min compile is cached as
  `uet_amr_realsense:<hash of that Dockerfile>`; only editing it recompiles. On x86 the apt packages are used.
  The host only needs the udev rules (`hardware/udev/`, installed by `setup`).
- **nvblox** (`nvblox_ros`, `nvblox_nav2`) comes from NVIDIA's apt repo inside the container. It takes its pose from the
  wheel+IMU EKF, not visual SLAM, and feeds Nav2's local costmap (`nvblox_nav2::NvbloxCostmapLayer`, loaded by name from
  `uet_amr_navigation/config/nav2.yaml`).

## People segmentation (optional)

With `use_detection:=true` nvblox leaves people out of the static map, so someone walking through the view does not leave a
ghost obstacle. The mask comes from `isaac_ros_unet` PeopleSemSegNet. Install the model once inside the container (it downloads
from NGC, builds a TensorRT engine for this GPU and asks you to accept NVIDIA's EULA; from the host use `docker exec -it`):

```bash
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_vanilla.sh --eula
```

The engine lands in `$ISAAC_ROS_WS/isaac_ros_assets/models/peoplesemsegnet/`. Then:
`ros2 launch uet_amr_bringup amr_bringup.launch.py use_detection:=true`. If the engine is missing, the launch stops and prints
the install command. nvblox then reads the colour-aligned depth, so the costmap sees only the colour camera's narrower field of
view. To feed your own mask pass `run_segmentation:=false` and publish it on `mask_image_topic`
(`uet_amr_perception/launch/nvblox.launch.py`).

## Troubleshooting

- **Linker "syntax error" on a `.so`** (e.g. `libgxf_core.so:1: syntax error`): the file is a Git LFS pointer. Install `git-lfs`,
  then `cd ros2/third_party/isaac_ros_common && git lfs install --local && git lfs pull` and rebuild.
- **`nvidia-smi` fails in the container**: check nvidia-container-toolkit and that `docker info` lists the `nvidia` runtime
  (Jetson: JetPack + `nvidia-docker2`).
- **`NvbloxCostmapLayer` not found**: `nvblox_nav2` isn't installed in the image; rebuild it with `scripts/amr --rebuild build`.
- **D435i frames dropped / low rate**: the host UDP buffer limits are not applied; run `scripts/amr setup` (installs
  `docker/90-uet-amr-dds.conf`), then `scripts/amr down` and restart.
- **Serial permission denied**: `setup` adds you to `dialout`; log out and back in.
- **Camera not found**: `rs-enumerate-devices` inside the container (`scripts/amr run rs-enumerate-devices`). Fix that before
  touching the ROS side.
