# =============================================================================
# UET AMR — Shared setup steps
# Sourced by setup_dev.sh and setup_prod.sh — not meant to be run directly
# =============================================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Isaac ROS packages this workspace can never build, regardless of GPU:
# isaac_ros_nvblox (the meta-package) and nvblox_examples_bringup both
# <depend> on NVIDIA's people-detection/segmentation model-install packages
# (isaac_ros_peoplenet_models_install etc.), which aren't vendored here -- we
# only use isaac_ros_nvblox's nvblox_ros/nvblox_nav2/nvblox_msgs packages, not
# the example bringup or its model downloads. Determined by walking the real
# package.xml dependency graph across ros2/src/third_party/ (94 packages).
ISAAC_ROS_ALWAYS_SKIP=(isaac_ros_nvblox nvblox_examples_bringup)

# Isaac ROS / NITROS packages that need CUDA Toolkit + NVIDIA's GXF binaries
# to build -- i.e. must be built inside the isaac_ros_common Docker image
# (see tools/setup_isaac_ros.sh), never natively. Skipped (with everything
# that transitively depends on them, via --packages-skip-by-dep) when no GPU
# is present. nvblox_nav2/nvblox_msgs/nvblox_ros_common have no CUDA
# dependency and are NOT in this list -- they build fine everywhere, so the
# Nav2 costmap plugin is always available (just inert without nvblox_ros
# actually running).
ISAAC_ROS_GPU_PACKAGES=(nvblox_ros isaac_ros_gxf isaac_ros_managed_nitros)

is_jetson() {
  [ -f /etc/nv_tegra_release ]
}

has_nvidia_gpu() {
  is_jetson && return 0
  command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1
}

# Initializes the git submodule at ros2/src/third_party/$1 (path relative to
# repo root), if not already checked out. Idempotent.
init_submodule() {
  local path="ros2/src/third_party/$1"
  cd "$REPO_ROOT"
  if [ -d "$path/.git" ]; then
    echo "  ($1 already checked out -- skipping)"
    return
  fi
  echo -e "${YELLOW}Initializing $1 submodule...${NC}"
  # --recursive: isaac_ros_nvblox nests its own submodule at
  # nvblox_ros/nvblox_core (the core C++/CUDA library) -- a plain --init
  # would leave that directory empty and break its CMake configure step.
  git submodule update --init --recursive "$path"
}

install_ros2_deps() {
  echo -e "${YELLOW}Installing ROS2 Humble dependencies...${NC}"
  sudo apt-get update -q

  local packages=(
    python3-pip
    ros-humble-nav2-bringup
    ros-humble-ros2-control
    ros-humble-ros2-controllers
    ros-humble-slam-toolbox
    ros-humble-diff-drive-controller
    ros-humble-joint-state-broadcaster
  )
  if is_jetson; then
    echo "  (Jetson detected -- skipping ros-humble-realsense2-camera, see setup_realsense_jetson)"
  else
    packages+=(ros-humble-realsense2-camera)
  fi

  sudo apt-get install -y -q "${packages[@]}"
}

setup_rplidar_udev() {
  init_submodule rplidar_ros

  local rules_src="$REPO_ROOT/ros2/src/third_party/rplidar_ros/scripts/rplidar.rules"
  if [ ! -f "$rules_src" ]; then
    echo "  (rplidar.rules not found at $rules_src -- skipping)"
    return
  fi

  # Check if running inside container/distrobox
  if [ -f /run/.containerenv ] || [ -f /.dockerenv ] || [ -n "$DISTROBOX_ENTERED" ]; then
    echo -e "${YELLOW}[Container detected] Skipping udev rule install inside container.${NC}"
    echo -e "Run this once on HOST to install udev rule:"
    echo -e "  sudo cp \"$rules_src\" /etc/udev/rules.d/rplidar.rules"
    echo -e "  sudo udevadm control --reload && sudo udevadm trigger"
    return
  fi

  echo -e "${YELLOW}Installing RPLidar udev rule (device -> /dev/rplidar)...${NC}"
  sudo cp "$rules_src" /etc/udev/rules.d/rplidar.rules
  sudo udevadm control --reload && sudo udevadm trigger
}

# On Jetson, pulls the realsense-ros ROS2 wrapper in as workspace source so it
# builds against whatever librealsense2 is installed on the system (built from
# source manually -- see docs/jetson_realsense_setup.md) instead of linking the
# incompatible apt librealsense2. No-op on non-Jetson machines, where the apt
# ros-humble-realsense2-camera package (installed by install_ros2_deps) is used
# as-is.
setup_realsense_jetson() {
  if ! is_jetson; then
    echo "  (Not a Jetson -- skipping RealSense source build)"
    return
  fi

  init_submodule realsense-ros
}

# $1: "true" (default) to include uet_amr_simulation (Gazebo sim deps),
#     "false" to skip it — for production/robot machines that don't run
#     simulation and may lack arm64 apt binaries for ros_gz_sim / ign_ros2_control.
build_workspace() {
  local include_simulation="${1:-true}"

  echo -e "${YELLOW}Installing ROS2 workspace dependencies...${NC}"
  cd "$REPO_ROOT/ros2"
  source /opt/ros/humble/setup.bash
  rosdep update --rosdistro=humble

  local skip_keys=()
  local skip_packages=()
  local skip_by_dep=()

  if [ "$include_simulation" = "false" ]; then
    skip_keys+=(ros_gz_sim ros_gz_bridge ign_ros2_control)
    skip_packages+=(uet_amr_simulation)
  fi
  if is_jetson; then
    # librealsense2 is built from source manually on Jetson (see
    # docs/jetson_realsense_setup.md); letting rosdep resolve it would apt-install
    # the incompatible generic librealsense2-dev on top of it and clobber the build.
    skip_keys+=(librealsense2)
  fi
  if [ -d "$REPO_ROOT/ros2/src/third_party/isaac_ros_nvblox" ]; then
    # --packages-skip only skips the named packages; --packages-skip-by-dep
    # additionally skips everything that transitively depends on them (see
    # ISAAC_ROS_ALWAYS_SKIP / ISAAC_ROS_GPU_PACKAGES above) -- both are needed
    # since a package that depends on a skipped one would otherwise fail.
    skip_packages+=("${ISAAC_ROS_ALWAYS_SKIP[@]}")
    skip_by_dep+=("${ISAAC_ROS_ALWAYS_SKIP[@]}")
    if ! has_nvidia_gpu; then
      skip_packages+=("${ISAAC_ROS_GPU_PACKAGES[@]}")
      skip_by_dep+=("${ISAAC_ROS_GPU_PACKAGES[@]}")
    fi
  fi

  if [ "${#skip_keys[@]}" -gt 0 ]; then
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "${skip_keys[*]}"
  else
    rosdep install --from-paths src --ignore-src -r -y
  fi

  echo -e "${YELLOW}Building ROS2 workspace...${NC}"
  local colcon_args=(build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
  [ "${#skip_packages[@]}" -gt 0 ] && colcon_args+=(--packages-skip "${skip_packages[@]}")
  [ "${#skip_by_dep[@]}" -gt 0 ] && colcon_args+=(--packages-skip-by-dep "${skip_by_dep[@]}")
  colcon "${colcon_args[@]}"
  source install/setup.bash
}

setup_bashrc() {
  if ! grep -q "UET_RE_AMR" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# UET AMR" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source $REPO_ROOT/ros2/install/setup.bash 2>/dev/null || true" >> ~/.bashrc
  fi
}
