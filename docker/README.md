# docker/

Only what the Isaac ROS environment needs. Everything is driven by `scripts/amr`; don't call NVIDIA's
`run_dev.sh` or `build_image_layers.sh` directly.

- `amr.env` — `ROS_DOMAIN_ID` (42) and `RMW_IMPLEMENTATION` (CycloneDDS), read by `scripts/amr` and `compose.zenoh.yml`.
  The zenoh bridge is Cyclone-based and Fast DDS services never reach it, so every machine talking to the robot needs the same
  RMW and domain. Change it, then `scripts/amr down`.
- `Dockerfile.uet_amr` — our top layer, stacked on NVIDIA's `isaac_ros_common` layers (and the RealSense layer on a Jetson).
  Bakes in the workspace's apt dependencies so they are not reinstalled in every container.
- `Dockerfile.uet_amr_realsense` — Jetson only. Swaps NVIDIA's librealsense 2.55.1 / realsense-ros 4.51.1-isaac for
  2.58.4 / 4.58.4 (2.55.1 spams IMU errors on D435i firmware 5.17.x). The ~15 min compile is tagged
  `uet_amr_realsense:<hash of this file>` and reused while that tag exists; only editing this file recompiles it.
- `.pkgxml/` (generated, gitignored) — stub `package.xml` files, the build context of `Dockerfile.uet_amr`, so it rebuilds only
  when a dependency changes.
- `dev-cyclonedds.xml` + `90-uet-amr-dds.conf` — 10 MB UDP receive buffer for CycloneDDS (D435i frames were dropped
  without it) and the host sysctl that allows it. `scripts/amr setup` installs the sysctl.
- `uet-amr.service` — systemd unit that starts bringup at boot; installed by `scripts/amr setup --robot`.
- `compose.zenoh.yml`, `zenoh-cyclonedds.xml`, `zenoh-bridge.json5` — optional
  [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) (pinned to 1.10.1) in its own host-network
  container: `docker compose -f docker/compose.zenoh.yml up -d` (`scripts/amr down` also stops it).
  The XML pins the bridge's DDS to loopback; without it the bridge saw no topics on a multi-NIC host.
  A remote PC connects with `-e tcp/<robot-ip>:7447`; open that port in the robot's firewall (`ufw allow 7447/tcp`).
  `zenoh-bridge.json5` stops the bridge creating (volatile) publishers on latched topics (`/map`, `/robot_description`,
  `/tf_static`); otherwise Foxglove subscribes to them volatile and never receives the map.

`scripts/amr` fetches `isaac_ros_common` (`ros2/third_party/`, pinned in `ros2/third_party.lock`) and writes NVIDIA's
config (`CONFIG_DOCKER_SEARCH_DIRS` -> this directory, plus the image key); see `scripts/lib/devenv.sh`.
If the `isaac_ros_common` pin moves, re-check `build_image_layers.sh` in that release.

Full workflow: [`docs/setup.md`](../docs/setup.md).
