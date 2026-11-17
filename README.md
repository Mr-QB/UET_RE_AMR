# UET_RE_AMR — Autonomous Mobile Robot

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Jetson%20%7C%20NVIDIA%20GPU%20PC-76b900)](docs/setup.md)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

Autonomous Mobile Robot (AMR) project developed by the Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi.

---

## Overview

UET_RE_AMR is an autonomous mobile robot platform designed for research and educational purposes. The system integrates a ROS2-based software stack for navigation and high-level control with MCU firmware for low-level motor actuation, sensor feedback, and power management, communicating over a direct UART link.

Key features include:
- Simultaneous Localization and Mapping (SLAM)
- Autonomous path planning and obstacle avoidance using Nav2
- Sensor integration including 2D LiDAR, IMU, and cameras
- GPU-accelerated 3D obstacle mapping (Isaac ROS nvblox) and optional people masking
- Hardware interface using ros2_control for real-time differential drive control

---

## Repository Structure

```
UET_RE_AMR/
├── ros2/src/
│   ├── uet_amr_bringup/        # Launch files, controller config, mode manager (/amr/set_mode, save_map, ...)
│   ├── uet_amr_description/    # URDF, meshes, RViz configs
│   ├── uet_amr_hardware/       # ros2_control hardware interface + its ros2_control xacro
│   ├── uet_amr_localization/   # EKF sensor fusion (wheel odom + IMU)
│   ├── uet_amr_maps/           # Saved maps, map cleaning, dashboard zones
│   ├── uet_amr_msgs/           # Custom messages and services
│   ├── uet_amr_navigation/     # Nav2 + slam_toolbox configuration
│   ├── uet_amr_perception/     # nvblox (GPU costmap) and people segmentation
│   └── uet_amr_teleop/         # Keyboard / joystick teleoperation and gate
├── ros2/third_party/            # Fetched by scripts/amr setup (gitignored, pins in ros2/third_party.lock): isaac_ros_common
├── firmware/
│   ├── amr_hoverboard_controller/  # STM32 motor control & wheel odometry firmware
│   └── amr_uart_bridge/            # ESP32 UART bridge for sensors/status
├── hardware/    # udev rules, BOM
├── docs/        # Setup and usage guides
├── scripts/     # amr: the one entry point (setup/build/shell/run/down), lib/
└── docker/      # Isaac ROS image layers, DDS settings, zenoh bridge, systemd unit
```

---

## Quick Start

Supported targets: an **NVIDIA Jetson** (the robot) and an **x86 PC with an NVIDIA GPU** (development). Everything runs in
a Docker container built on [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS); see [Setup](docs/setup.md).

```bash
git clone https://github.com/UET-RE/UET_RE_AMR.git
cd UET_RE_AMR
./scripts/amr setup            # add --robot on the Jetson to start the robot at boot
./scripts/amr build
./scripts/amr run ros2 launch uet_amr_bringup amr_bringup.launch.py
```

Mapping, navigation and teleop: [Using the robot](docs/getting_started.md).

---

## Documentation

| Document | Description |
|---|---|
| [Setup](docs/setup.md) | Jetson / NVIDIA PC setup, Isaac ROS image, RealSense, people segmentation, troubleshooting |
| [Using the robot](docs/getting_started.md) | Bringup, mapping, navigation, teleop, known firmware-driven quirks |
| [D435i Guide](docs/d435i_guide.md) | The RealSense D435i as configured on this robot |
| [Docker](docker/README.md) | Image layers, DDS settings, zenoh bridge, boot service |
| [Navigation Package](ros2/src/uet_amr_navigation/README.md) | Nav2, slam_toolbox, workarounds |
| [Maps Package](ros2/src/uet_amr_maps/README.md) | Saved maps, zones (no-go / slow areas) |
| [Hardware Docs](hardware/README.md) | Controllers, power, BOM |
| [Hoverboard Controller Firmware](firmware/amr_hoverboard_controller/README.md) | STM32 motor control firmware |
| [UART Bridge Firmware](firmware/amr_uart_bridge/README.md) | ESP32 UART bridge for sensors/status |

---

## Team & Contribution Workflow

For details on branching, commit messages, and contribution guidelines, see [`CONTRIBUTING.md`](CONTRIBUTING.md)

## Acknowledgements

UET_RE_AMR builds on the work of several open-source projects and communities:

- **[linorobot2](https://github.com/linorobot/linorobot2)** — the overall system architecture and ROS2 integration approach for this project were inspired by linorobot2's design.
- **[hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)** by [EFeru](https://github.com/EFeru) — the base for `firmware/amr_hoverboard_controller`, adapted with a custom communication protocol and motor control logic for this platform. Licensed under GPL-3.0; see [`firmware/amr_hoverboard_controller/LICENSE`](firmware/amr_hoverboard_controller/LICENSE).
- **[rplidar_ros](https://github.com/Slamtec/rplidar_ros)** by Slamtec — LiDAR driver (installed from apt in the image).
- **[Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS)** by NVIDIA — `nvblox_ros` provides GPU-accelerated 3D reconstruction feeding `uet_amr_navigation`'s Nav2 local costmap via `nvblox_nav2` feeding the Nav2 local costmap; see [Setup](docs/setup.md).
- **[Nav2](https://github.com/ros-planning/navigation2)**, **[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)**, and **[robot_localization](https://github.com/cra-ros-pkg/robot_localization)** — the ROS2 navigation, SLAM, and sensor-fusion stacks this project is built on top of.


## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

Copyright (c) 2026 Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi.
