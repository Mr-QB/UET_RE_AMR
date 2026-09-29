#!/bin/bash
# =============================================================================
# UET AMR — Isaac ROS + Docker Dev Environment Setup
#
# Initializes the isaac_ros_common / isaac_ros_nitros / isaac_ros_nvblox
# submodules under ros2/src/third_party/ (isaac_ros_nitros is nvblox_ros's
# NITROS/GXF transport dependency -- a separate NVIDIA repo, not part of
# isaac_ros_nvblox itself), wires up UET AMR's container overrides (serial
# device, X11, host networking), then launches isaac_ros_common's own
# scripts/run_dev.sh, which builds the GPU dev image (x86_64 dGPU or
# Jetson/L4T, auto-detected) and drops you into it.
#
# Separate from tools/setup_dev.sh / setup_prod.sh (native, non-Docker flows) —
# run this instead when you want to build/run isaac_ros_nvblox.
#
# Requires Docker + nvidia-container-toolkit on x86_64, or JetPack on Jetson.
# See docs/isaac_ros_setup.md.
# =============================================================================

set -e
echo "🤖 UET AMR — Setting up Isaac ROS + Docker dev environment..."

source "$(dirname "$0")/common.sh"

if ! has_nvidia_gpu; then
  echo -e "${YELLOW}Warning: no NVIDIA GPU detected on this host.${NC}"
  echo "  isaac_ros_nvblox needs a GPU to build/run. You can still fetch"
  echo "  sources and build the image here; native (non-Docker) builds skip"
  echo "  GPU packages automatically (see tools/common.sh: ISAAC_ROS_GPU_PACKAGES)."
fi

echo -e "${YELLOW}[1/3] Initializing Isaac ROS submodules...${NC}"
init_submodule isaac_ros_common
init_submodule isaac_ros_nitros
init_submodule isaac_ros_nvblox

RUN_DEV_SH="$REPO_ROOT/ros2/src/third_party/isaac_ros_common/scripts/run_dev.sh"
if [ ! -x "$RUN_DEV_SH" ]; then
  echo "isaac_ros_common's run_dev.sh not found or not executable at:"
  echo "  $RUN_DEV_SH"
  echo "The vendored isaac_ros_common layout may have changed upstream --"
  echo "check the release-3.2 pin in .gitmodules against current docs."
  exit 1
fi

echo -e "${YELLOW}[2/3] Applying UET AMR container overrides (serial device)...${NC}"
# run_dev.sh reads extra `docker run` args from ~/.isaac_ros_dev-dockerargs
# (or a copy next to itself) -- see isaac_ros_common/scripts/run_dev.sh.
# X11/DISPLAY, ROS_DOMAIN_ID, and --network host are already handled by
# run_dev.sh's own defaults; the dockerargs file only adds the serial device.
cp "$REPO_ROOT/docker/isaac_ros/config/isaac_ros_dev-dockerargs" \
   "$HOME/.isaac_ros_dev-dockerargs"

echo -e "${YELLOW}[3/3] Building (if needed) and launching the Isaac ROS dev container...${NC}"
echo "  Workspace: $REPO_ROOT/ros2 -> mounted at /workspaces/isaac_ros-dev"
echo ""
echo "Once inside the container:"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install \\"
echo "    --packages-skip isaac_ros_nvblox nvblox_examples_bringup \\"
echo "    --packages-skip-by-dep isaac_ros_nvblox nvblox_examples_bringup"
echo "  # (those two need NVIDIA's people-detection model-install packages,"
echo "  #  which aren't vendored here -- we only use nvblox_ros/nvblox_nav2)"
echo "  source install/setup.bash"
echo "  ros2 launch uet_amr_navigation navigation.launch.py use_nvblox:=true"
echo ""
echo "Separately, start the micro-ROS agent on the host (see docker/isaac_ros/README.md):"
echo "  docker run --rm --network host --device=/dev/ttyUSB0 \\"
echo "    microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 --baudrate 115200"
echo ""

exec "$RUN_DEV_SH" -d "$REPO_ROOT/ros2"
