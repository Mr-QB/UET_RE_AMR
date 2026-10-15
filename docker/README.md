# docker/

Only what the Isaac ROS dev environment needs; no other Docker files live in this repo.

- `Dockerfile.uet_amr_realsense` — Jetson only. Swaps NVIDIA's librealsense 2.55.1 / realsense-ros 4.51.1-isaac
  for 2.58.4 / 4.58.4 (2.55.1 spams IMU errors on D435i firmware 5.17.x). The ~15 min compile is tagged
  `uet_amr_realsense:<hash of this file>` and reused while that tag exists, so it survives `docker builder prune`
  and workspace dependency changes; only editing this file (e.g. the version pins) recompiles it.
- `Dockerfile.uet_amr` — our top layer, stacked on NVIDIA's `isaac_ros_common` layers (and the RealSense layer
  above on a Jetson). It bakes in the workspace's apt dependencies so they are not reinstalled in every container.
- `isaac_ros_dev-dockerargs` — extra `docker run` args for NVIDIA's `run_dev.sh` (the serial device). Copied
  to `~/.isaac_ros_dev-dockerargs`, dropping any `--device` that isn't plugged in.
- `.pkgxml/` (generated, gitignored) — stub `package.xml` files (name + dependencies), used as this layer's build context so it only
  rebuilds when a dependency changes (version/metadata edits are stripped out).
- `compose.zenoh.yml`, `zenoh-cyclonedds.xml` — optional [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
  (`:latest`, v1.10.1 when tested) in its own host-network container. Start it once with `docker compose -f docker/compose.zenoh.yml up -d` (it restarts itself; `docker-dev down` also stops it).
  The XML pins the bridge's DDS to loopback; without it the bridge saw no topics on a multi-NIC host.
  A remote PC connects with `-e tcp/<robot-ip>:7447`; open that port in the robot's firewall (`ufw allow 7447/tcp`).

Don't call `run_dev.sh` or `build_image_layers.sh` directly. Use `scripts/setup.sh docker-dev`, which fetches
`isaac_ros_common` (`ros2/third_party/`, pinned in `ros2/third_party.lock`), writes NVIDIA's config
(`CONFIG_DOCKER_SEARCH_DIRS` -> this directory, plus the image key) and runs them. See `scripts/lib/devenv.sh`.

Full workflow: [`docs/isaac_ros_setup.md`](../docs/isaac_ros_setup.md).

If the `isaac_ros_common` pin moves, re-check `run_dev.sh` in that release — NVIDIA has changed the dockerargs
mechanism before.
