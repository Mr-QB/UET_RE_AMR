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

build_workspace() {
  echo -e "${YELLOW}Installing ROS2 workspace dependencies...${NC}"
  cd "$REPO_ROOT/ros2"
  source /opt/ros/humble/setup.bash
  rosdep update --rosdistro=humble
  rosdep install --from-paths src --ignore-src -r -y

  echo -e "${YELLOW}Building ROS2 workspace...${NC}"
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
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
