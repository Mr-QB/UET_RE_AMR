#!/bin/bash
# =============================================================================
# UET AMR — Production Environment Setup Script
# ROS2 workspace only — no PlatformIO / firmware toolchain
# Run once on a robot / deployment machine (Ubuntu 22.04)
# =============================================================================

set -e
echo "🤖 UET AMR — Setting up production environment..."

source "$(dirname "$0")/common.sh"

echo -e "${YELLOW}[1/5] Installing ROS2 Humble dependencies...${NC}"
install_ros2_deps

echo -e "${YELLOW}[2/5] Initializing third-party submodules...${NC}"
init_submodules

echo -e "${YELLOW}[3/5] Installing RPLidar udev rule...${NC}"
setup_rplidar_udev

echo -e "${YELLOW}[4/5] Installing ROS2 workspace dependencies...${NC}"
echo -e "${YELLOW}[5/5] Building ROS2 workspace...${NC}"
build_workspace false

setup_bashrc

echo ""
echo -e "${GREEN}✅ Production setup complete!${NC}"
echo ""
echo "Next steps:"
echo "  source ~/.bashrc"
echo "  ros2 launch uet_amr_bringup amr_bringup.launch.py"
