#!/bin/bash
# =============================================================================
# UET AMR — Developer Environment Setup Script
# Full setup: ROS2 workspace + low-level firmware toolchain (PlatformIO)
# Run once on a new Ubuntu 22.04 machine / WSL2
# =============================================================================

set -e
echo "🤖 UET AMR — Setting up developer environment..."

source "$(dirname "$0")/common.sh"

echo -e "${YELLOW}[1/5] Installing ROS2 Humble dependencies...${NC}"
install_ros2_deps

echo -e "${YELLOW}[2/5] Installing RPLidar udev rule...${NC}"
setup_rplidar_udev

echo -e "${YELLOW}[3/5] Installing ROS2 workspace dependencies...${NC}"
echo -e "${YELLOW}[4/5] Building ROS2 workspace...${NC}"
build_workspace

# ----------- PlatformIO -----------
echo -e "${YELLOW}[5/5] Installing PlatformIO...${NC}"
pip install --quiet platformio

setup_bashrc

echo ""
echo -e "${GREEN}✅ Developer setup complete!${NC}"
echo ""
echo "Next steps:"
echo "  source ~/.bashrc"
echo "  ros2 launch uet_amr_simulation simulation.launch.py"
