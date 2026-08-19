# =============================================================================
# UET AMR — Shared setup steps
# Sourced by setup_dev.sh and setup_prod.sh — not meant to be run directly
# =============================================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

is_jetson() {
  [ -f /etc/nv_tegra_release ]
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

# Initializes the rplidar_ros submodule and installs its vendored udev rule so
# the sensor always enumerates as the fixed /dev/rplidar symlink (matches
# uet_amr_bringup/launch/lidar.launch.py's serial_port param) instead of a
# shifting /dev/ttyUSBX.
setup_rplidar_udev() {
  echo -e "${YELLOW}Initializing rplidar_ros submodule...${NC}"
  cd "$REPO_ROOT"
  git submodule update --init --recursive ros2/src/third_party/rplidar_ros

  echo -e "${YELLOW}Installing RPLidar udev rule (device -> /dev/rplidar)...${NC}"
  local rules_src="$REPO_ROOT/ros2/src/third_party/rplidar_ros/scripts/rplidar.rules"
  if [ ! -f "$rules_src" ]; then
    echo "  (rplidar.rules not found at $rules_src -- skipping)"
    return
  fi
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

  echo -e "${YELLOW}Fetching realsense-ros (ROS2 wrapper) source...${NC}"
  cd "$REPO_ROOT"
  git submodule update --init ros2/src/third_party/realsense-ros
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

  local skip_keys=""
  if [ "$include_simulation" = "false" ]; then
    skip_keys="ros_gz_sim ros_gz_bridge ign_ros2_control"
  fi
  if is_jetson; then
    # librealsense2 is built from source manually on Jetson (see
    # docs/jetson_realsense_setup.md); letting rosdep resolve it would apt-install
    # the incompatible generic librealsense2-dev on top of it and clobber the build.
    skip_keys="$skip_keys librealsense2"
  fi

  if [ -n "$skip_keys" ]; then
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "$skip_keys"
  else
    rosdep install --from-paths src --ignore-src -r -y
  fi

  echo -e "${YELLOW}Building ROS2 workspace...${NC}"
  if [ "$include_simulation" = "false" ]; then
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-skip uet_amr_simulation
  else
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  fi
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
