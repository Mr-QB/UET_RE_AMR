# docker/

Only what the Isaac ROS dev environment needs; no other Docker files live in this repo.

- `Dockerfile.uet_amr` — our one layer, stacked on NVIDIA's `isaac_ros_common` layers (prebuilt base, plus
  the RealSense layer on a Jetson). It bakes in the workspace's apt dependencies so they are not reinstalled
  in every container.
- `isaac_ros_dev-dockerargs` — extra `docker run` args for NVIDIA's `run_dev.sh` (the serial device). Copied
  to `~/.isaac_ros_dev-dockerargs`, dropping any `--device` that isn't plugged in.
- `.pkgxml/` (generated, gitignored) — the `package.xml` files, used as this layer's build context so it only
  rebuilds when a dependency changes.
- `compose.zenoh.yml`, `zenoh-cyclonedds.xml` — optional [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
  (`:latest`, v1.10.1 when tested) in its own host-network container. Start it with `scripts/setup.sh docker-dev --with-zenoh <subcommand>` (`docker-dev down` stops it) or `docker compose -f docker/compose.zenoh.yml up -d`.
  The XML pins the bridge's DDS to loopback; without it the bridge saw no topics on a multi-NIC host.
  A remote PC connects with `-e tcp/<robot-ip>:7447`; open that port in the robot's firewall (`ufw allow 7447/tcp`).

Don't call `run_dev.sh` or `build_image_layers.sh` directly. Use `scripts/setup.sh docker-dev`, which fetches
`isaac_ros_common` (`ros2/third_party/`, pinned in `ros2/third_party.lock`), writes NVIDIA's config
(`CONFIG_DOCKER_SEARCH_DIRS` -> this directory, plus the image key) and runs them. See `scripts/lib/devenv.sh`.

Full workflow: [`docs/isaac_ros_setup.md`](../docs/isaac_ros_setup.md).

If the `isaac_ros_common` pin moves, re-check `run_dev.sh` in that release — NVIDIA has changed the dockerargs
mechanism before.
