#!/bin/bash
# Docker entrypoint for ROS2 container
set -e

source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash

exec "$@"
