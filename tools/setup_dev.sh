#!/bin/bash
# =============================================================================
# UET AMR — Developer Environment Setup Script
# Full setup: ROS2 workspace + low-level firmware toolchain (PlatformIO, micro-ROS)
# Run once on a new Ubuntu 22.04 machine / WSL2
# =============================================================================

set -e
echo "🤖 UET AMR — Setting up developer environment..."

source "$(dirname "$0")/common.sh"

echo -e "${YELLOW}[1/6] Installing ROS2 Humble dependencies...${NC}"
install_ros2_deps

echo -e "${YELLOW}[2/6] Installing RPLidar udev rule...${NC}"
setup_rplidar_udev

echo -e "${YELLOW}[3/6] Installing ROS2 workspace dependencies...${NC}"
echo -e "${YELLOW}[4/6] Building ROS2 workspace...${NC}"
build_workspace

# ----------- PlatformIO -----------
echo -e "${YELLOW}[5/6] Installing PlatformIO...${NC}"
pip install --quiet platformio

# ----------- micro-ROS agent -----------
echo -e "${YELLOW}[6/6] Installing micro-ROS agent...${NC}"
pip install --quiet micro-ros-agent 2>/dev/null || \
  echo "  (Optional: install micro-ROS agent manually if needed)"

setup_bashrc

echo ""
echo -e "${GREEN}✅ Developer setup complete!${NC}"
echo ""
echo "Next steps:"
echo "  source ~/.bashrc"
echo "  ros2 launch uet_amr_simulation amr_simulation.launch.py"
