#!/bin/bash
# Bare metal, robot / deployment machine: ROS 2 workspace only, no simulation, no PlatformIO.
# On a Jetson this also builds the RSUSB librealsense.
set -e
source "$(dirname "$0")/../common.sh"

echo "🤖 UET AMR — native production setup"

echo -e "${YELLOW}[1/4] ROS 2 Humble dependencies...${NC}"
install_ros2_deps

echo -e "${YELLOW}[2/4] udev rules (RPLidar, RealSense)...${NC}"
install_udev_rules

echo -e "${YELLOW}[3/4] RealSense (Jetson only)...${NC}"
setup_realsense_jetson

echo -e "${YELLOW}[4/4] Building ROS 2 workspace...${NC}"
build_workspace false

setup_bashrc

echo -e "\n${GREEN}✅ Production setup complete!${NC}"
echo "  source ~/.bashrc"
echo "  ros2 launch uet_amr_bringup amr_bringup.launch.py"
