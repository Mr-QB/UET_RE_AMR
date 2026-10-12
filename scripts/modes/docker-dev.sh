#!/bin/bash
# Docker development environment, built on NVIDIA's isaac_ros_common layering:
#   NVIDIA base (prebuilt where published) [+ RealSense layer on Jetson] + docker/Dockerfile.uet_amr
# ros2/ is mounted at /workspaces/isaac_ros-dev, so build/ install/ log/ persist on the host.
#
#   docker-dev                interactive shell via NVIDIA's run_dev.sh (container removed on exit)
#
# Long-lived container for a fast "git pull, rebuild" loop (nothing is reinstalled or recompiled
# except your own changed code; --symlink-install means launch/config/Python edits need no build):
#   docker-dev build          update image if needed, incremental colcon build
#   docker-dev shell          shell in the long-lived container
#   docker-dev run <cmd...>   run a command in it, e.g.  docker-dev run ros2 launch uet_amr_bringup amr_bringup.launch.py
#   docker-dev down           remove the container (build/ install/ stay on disk)
set -e
source "$(dirname "$0")/../common.sh"

NAME=uet_amr_dev
WS=/workspaces/isaac_ros-dev
SRC='source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash'

# The image ID changes on every `docker build` (build metadata), even when every layer is
# identical, so compare a fingerprint of the layer list instead.
image_fp() { docker image inspect -f '{{json .RootFS.Layers}}' $DEV_IMAGE | sha256sum | cut -c1-16; }

container_ok() {  # running, and created from an image with the same layers
  [ "$(docker inspect -f '{{.State.Running}}' $NAME 2>/dev/null)" = true ] &&
  [ "$(docker inspect -f '{{index .Config.Labels "uet_amr.image"}}' $NAME)" = "$(image_fp)" ]
}

start_container() {
  docker rm -f $NAME >/dev/null 2>&1 || true
  local args=(
    -d --restart unless-stopped --name $NAME --label uet_amr.image="$(image_fp)"
    --privileged --network host --ipc=host --runtime nvidia
    --user "$(id -u):$(id -g)" -e HOME=/tmp
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_DOMAIN_ID -e DISPLAY
    -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev -v /etc/localtime:/etc/localtime:ro
    -v "$REPO_ROOT/ros2:$WS" -w $WS
  )
  if [ "$(uname -m)" = aarch64 ]; then   # the Jetson mounts NVIDIA's run_dev.sh adds
    args+=(-e NVIDIA_VISIBLE_DEVICES="nvidia.com/gpu=all,nvidia.com/pva=all" --pid=host
      -v /usr/bin/tegrastats:/usr/bin/tegrastats
      -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra
      -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api
      -v /usr/share/vpi3:/usr/share/vpi3)
  fi
  [ -d /dev/input ] && args+=(-v /dev/input:/dev/input)
  docker run "${args[@]}" $DEV_IMAGE sleep infinity >/dev/null
}

in_container() { local t=-i; [ -t 0 ] && t=-it; docker exec $t -w $WS $NAME bash -c "$SRC; $1" _ "${@:2}"; }

up() {
  dev_build_image
  container_ok || { echo -e "${YELLOW}Starting $NAME...${NC}"; start_container; }
}

build() {
  echo -e "${YELLOW}Incremental build...${NC}"
  in_container 'colcon build --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release'
}

case "${1:-}" in
  "")
    echo "🤖 UET AMR — Docker development environment"
    has_nvidia_gpu || echo -e "${YELLOW}Warning: no NVIDIA GPU detected; nvblox needs one to run.${NC}"
    install_udev_rules || echo -e "${YELLOW}Warning: udev rules not installed (needs sudo); continuing.${NC}"
    dev_prepare
    exec "$ISAAC_SCRIPTS/run_dev.sh" -d "$REPO_ROOT/ros2"
    ;;
  build) up; build ;;
  shell) up; in_container 'exec bash -i' ;;
  run)   shift; up; in_container 'exec "$@"' "$@" ;;
  down)  docker rm -f $NAME ;;
  *)     sed -n 2,15p "$0"; exit 1 ;;
esac
