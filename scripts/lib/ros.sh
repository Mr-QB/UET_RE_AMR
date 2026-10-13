# ROS 2 Humble: apt dependencies, workspace build, shell setup.

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
  packages+=(ros-humble-realsense2-camera)  # on Jetson its librealsense is replaced, see setup_realsense_jetson

  sudo apt-get install -y -q "${packages[@]}"
}

# $1: "true" (default) to include uet_amr_simulation (Gazebo sim deps),
#     "false" to skip it — for production/robot machines that don't run
#     simulation and may lack arm64 apt binaries for ros_gz_sim / ign_ros2_control.
build_workspace() {
  local include_simulation="${1:-true}"

  fetch_third_party rplidar_ros   # driver source, built with the workspace

  echo -e "${YELLOW}Installing ROS2 workspace dependencies...${NC}"
  cd "$REPO_ROOT/ros2"
  source /opt/ros/humble/setup.bash
  rosdep update --rosdistro=humble

  local skip_keys=()
  local skip_packages=()

  if [ "$include_simulation" = "false" ]; then
    skip_keys+=(ros_gz_sim ros_gz_bridge ign_ros2_control)
    skip_packages+=(uet_amr_simulation)
  fi
  if [ "${#skip_keys[@]}" -gt 0 ]; then
    rosdep install --from-paths src third_party --ignore-src -r -y --skip-keys "${skip_keys[*]}"
  else
    rosdep install --from-paths src third_party --ignore-src -r -y
  fi

  echo -e "${YELLOW}Building ROS2 workspace...${NC}"
  local colcon_args=(build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
  [ "${#skip_packages[@]}" -gt 0 ] && colcon_args+=(--packages-skip "${skip_packages[@]}")
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
