#!/bin/bash
# =============================================================================
# UET AMR — Environment Setup Script
# Run once on a new Ubuntu 22.04 machine / WSL2
# =============================================================================

set -e
echo "🤖 UET AMR — Setting up development environment..."

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ----------- ROS2 Humble -----------
echo -e "${YELLOW}[1/5] Installing ROS2 Humble dependencies...${NC}"
sudo apt-get update -q
sudo apt-get install -y -q \
  ros-humble-nav2-bringup \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-slam-toolbox \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-realsense2-camera

# ----------- Third-party modules -----------
echo -e "${YELLOW}[2/6] Initializing third-party submodules...${NC}"
cd "$(dirname "$0")/.."
git submodule update --init --recursive ros2/src/third_party

# ----------- ROS2 workspace -----------
echo -e "${YELLOW}[3/6] Installing ROS2 workspace dependencies...${NC}"
cd "$(dirname "$0")/../ros2"
source /opt/ros/humble/setup.bash
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y

# ----------- Build -----------
echo -e "${YELLOW}[4/6] Building ROS2 workspace...${NC}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# ----------- PlatformIO -----------
echo -e "${YELLOW}[5/6] Installing PlatformIO...${NC}"
pip install --quiet platformio

# ----------- micro-ROS agent -----------
echo -e "${YELLOW}[6/6] Installing micro-ROS agent...${NC}"
pip install --quiet micro-ros-agent 2>/dev/null || \
  echo "  (Optional: install micro-ROS agent manually if needed)"

# ----------- .bashrc setup -----------
if ! grep -q "UET_RE_AMR" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# UET AMR" >> ~/.bashrc
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "source $(pwd)/install/setup.bash 2>/dev/null || true" >> ~/.bashrc
fi

echo ""
echo -e "${GREEN}✅ Setup complete!${NC}"
echo ""
echo "Next steps:"
echo "  source ~/.bashrc"
echo "  ros2 launch uet_amr_bringup amr_simulation.launch.py"
