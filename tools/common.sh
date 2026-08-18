# =============================================================================
# UET AMR — Shared setup steps
# Sourced by setup_dev.sh and setup_prod.sh — not meant to be run directly
# =============================================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

install_ros2_deps() {
  echo -e "${YELLOW}Installing ROS2 Humble dependencies...${NC}"
  sudo apt-get update -q
  sudo apt-get install -y -q \
    python3-pip \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-slam-toolbox \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-realsense2-camera
}

init_submodules() {
  echo -e "${YELLOW}Initializing third-party submodules...${NC}"
  cd "$REPO_ROOT"
  git submodule update --init --recursive ros2/src/third_party
}

# Installs the vendored RPLidar udev rule so the sensor always enumerates as
# the fixed /dev/rplidar symlink (matches uet_amr_bringup/launch/lidar.launch.py's
# serial_port param) instead of a shifting /dev/ttyUSBX. Requires init_submodules
# to have run first, since the rule ships inside the rplidar_ros submodule.
setup_rplidar_udev() {
  echo -e "${YELLOW}Installing RPLidar udev rule (device -> /dev/rplidar)...${NC}"
  local rules_src="$REPO_ROOT/ros2/src/third_party/rplidar_ros/scripts/rplidar.rules"
  if [ ! -f "$rules_src" ]; then
    echo "  (rplidar.rules not found at $rules_src -- skipping, run init_submodules first)"
    return
  fi
  sudo cp "$rules_src" /etc/udev/rules.d/rplidar.rules
  sudo udevadm control --reload && sudo udevadm trigger
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

  if [ "$include_simulation" = "false" ]; then
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys "ros_gz_sim ros_gz_bridge ign_ros2_control"
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
