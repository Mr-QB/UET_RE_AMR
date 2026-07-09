#!/bin/bash
# Start micro-ROS agent to bridge MCU firmware with ROS2

SERIAL_PORT=${1:-/dev/ttyUSB0}
BAUD_RATE=${2:-115200}

echo "🔌 Starting micro-ROS agent on $SERIAL_PORT @ ${BAUD_RATE} baud..."
echo "   Make sure firmware is flashed and MCU is connected."
echo "   Press Ctrl+C to stop."
echo ""

# Try Docker first (more reliable)
if command -v docker &> /dev/null; then
  docker run -it --rm \
    --device="$SERIAL_PORT" \
    --net=host \
    microros/micro-ros-agent:humble \
    serial --dev "$SERIAL_PORT" --baudrate "$BAUD_RATE"
else
  # Fallback: native
  ros2 run micro_ros_agent micro_ros_agent serial \
    --dev "$SERIAL_PORT" \
    --baudrate "$BAUD_RATE"
fi
