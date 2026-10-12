#!/bin/bash
# Bare metal, developer machine: ROS 2 workspace + firmware toolchain (PlatformIO).
# Ubuntu 22.04 / WSL2. On a Jetson this also builds the RSUSB librealsense.
set -e
source "$(dirname "$0")/../common.sh"

echo "🤖 UET AMR — native developer setup"

echo -e "${YELLOW}[1/5] ROS 2 Humble dependencies...${NC}"
install_ros2_deps

echo -e "${YELLOW}[2/5] udev rules (RPLidar, RealSense)...${NC}"
install_udev_rules

echo -e "${YELLOW}[3/5] RealSense (Jetson only)...${NC}"
setup_realsense_jetson

echo -e "${YELLOW}[4/5] Building ROS 2 workspace...${NC}"
build_workspace

echo -e "${YELLOW}[5/5] PlatformIO...${NC}"
pip install --quiet platformio

setup_bashrc

echo -e "\n${GREEN}✅ Developer setup complete!${NC}"
echo "  source ~/.bashrc"
echo "  ros2 launch uet_amr_simulation simulation.launch.py"
