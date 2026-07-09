# Communication Protocol Specification: Firmware <-> ROS2

This document defines the communication interfaces and serial packet formats used to connect the low-level microcontroller (MCU) firmware with the high-level ROS2 software stack.

---

## Overview

The primary communication framework for the UET AMR platform is micro-ROS, which uses a client-agent architecture.

```
+------------------+         +------------------+         +------------------+
|    ROS2 Host     |  UART   | micro-ROS Agent  |  UART   |  Microcontroller |
|   (Ubuntu PC)    |<------->|   (USB-Serial)   |<------->|  (STM32 / ESP32) |
+------------------+         +------------------+         +------------------+
```

---

## micro-ROS Topics

The micro-ROS client library running on the MCU publishes and subscribes to the following topics:

| Direction | Topic | Message Type | Frequency (Hz) |
|---|---|---|---|
| MCU -> ROS2 | `/wheel_odom` | `uet_amr_msgs/msg/WheelOdom` | 50 |
| MCU -> ROS2 | `/battery/status` | `uet_amr_msgs/msg/BatteryStatus` | 1 |
| MCU -> ROS2 | `/motors/status` | `uet_amr_msgs/msg/MotorStatus` | 10 |
| MCU -> ROS2 | `/imu/data_raw` | `sensor_msgs/msg/Imu` | 100 |
| ROS2 -> MCU | `/cmd_vel` | `geometry_msgs/msg/Twist` | 20 |

---

## Running the micro-ROS Agent

To initialize communication between the MCU and the host, run the micro-ROS agent using one of the following methods:

### Option 1: Docker (Recommended)
```bash
docker run -it --rm --net=host \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0 -b 115200
```

### Option 2: Native ROS2 Command
```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 --baudrate 115200
```

### Option 3: Script Invocation
```bash
./tools/start_microros_agent.sh
```

---

## Fallback Packet Format (Custom UART)

If micro-ROS is not used, communication falls back to a custom, lightweight binary protocol.

### Command Packet (ROS2 -> MCU)

All values are transmitted in little-endian format.

```
+------------+------------+---------------+-------------------------------------+------------+
| Header (1B)| Command(1B)| Data Length(1B)| Data (N Bytes)                      | Checksum(1B)|
+------------+------------+---------------+-------------------------------------+------------+
|    0xAA    |    CMD     |    LENGTH     | ...                                 |    CRC8    |
+------------+------------+---------------+-------------------------------------+------------+
```

- **Set Wheel Velocities** (CMD = `0x01`):
  - Data: Left Wheel Velocity (float32, 4 bytes), Right Wheel Velocity (float32, 4 bytes).
  - Total Data Length: 8 bytes.

### Feedback Packet (MCU -> ROS2)

```
+------------+------------+---------------+-------------------------------------+------------+
| Header (1B)| Type (1B)  | Data Length(1B)| Data (N Bytes)                      | Checksum(1B)|
+------------+------------+---------------+-------------------------------------+------------+
|    0xBB    |    TYPE    |    LENGTH     | ...                                 |    CRC8    |
+------------+------------+---------------+-------------------------------------+------------+
```

- **Odometry Feedback** (TYPE = `0x01`):
  - Data: Left Encoder Ticks (int32, 4 bytes), Right Encoder Ticks (int32, 4 bytes).
  - Total Data Length: 8 bytes.
- **Battery Status** (TYPE = `0x02`):
  - Data: Voltage (float32, 4 bytes), Current Draw (float32, 4 bytes), State of Charge (uint8, 1 byte).
  - Total Data Length: 9 bytes.

---

## Loop Timing Constraints

| Task | Execution Period | Update Frequency | Action on Timeout |
|---|---|---|---|
| PID Control Loop | 10 ms | 100 Hz | Halt motors on controller error |
| Odometry Publishing | 20 ms | 50 Hz | None |
| Battery Diagnostics | 1000 ms | 1 Hz | Log warning to ROS2 console |
| High-level Safety Heartbeat | 500 ms | - | Stop motors if no `/cmd_vel` received |
