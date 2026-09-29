# UET_RE_AMR — Autonomous Mobile Robot

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Docker-lightgrey)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

Autonomous Mobile Robot (AMR) project developed by the Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi.

---

## Overview

UET_RE_AMR is an autonomous mobile robot platform designed for research and educational purposes. The system integrates a ROS2-based software stack for navigation and high-level control with MCU firmware for low-level motor actuation, sensor feedback, and power management, communicating over a direct UART link.

Key features include:
- Simultaneous Localization and Mapping (SLAM)
- Autonomous path planning and obstacle avoidance using Nav2
- Sensor integration including 2D LiDAR, IMU, and cameras
- Hardware interface using ros2_control for real-time differential drive control

---

## Repository Structure

```
UET_RE_AMR/
├── ros2/src/
│   ├── uet_amr_bringup/        # Top-level launch files (simulation & hardware)
│   ├── uet_amr_description/    # URDF, meshes, and RViz configs
│   ├── uet_amr_hardware/       # ros2_control hardware interface
│   ├── uet_amr_localization/   # EKF sensor fusion (wheel odom + IMU)
│   ├── uet_amr_msgs/           # Custom messages, services, and actions
│   ├── uet_amr_navigation/     # Nav2 stack + slam_toolbox configuration
│   ├── uet_amr_simulation/     # Gazebo worlds and simulation launch files
│   ├── uet_amr_teleop/          # Keyboard / joystick teleoperation
│   └── third_party/             # Vendored packages (git submodules):
│                                 #   rplidar_ros, realsense-ros,
│                                 #   isaac_ros_common, isaac_ros_nvblox
├── firmware/
│   ├── amr_hoverboard_controller/  # STM32 motor control & wheel odometry firmware
│   └── amr_uart_bridge/            # ESP32 UART bridge for sensors/status
├── hardware/    # Schematics, PCB layouts, CAD models, BOM
├── docs/        # Technical documentation and guides
├── tools/       # Setup and utility scripts (dev/prod/Isaac ROS environment setup)
└── docker/      # Isaac ROS GPU dev container (docker/isaac_ros/)
```

---

## Quick Start

### Prerequisites

To run the ROS2 stack natively, you need Ubuntu 22.04 and ROS2 Humble:

```bash
# Install core dependencies
sudo apt update
sudo apt install ros-humble-desktop ros-humble-nav2-bringup
```

Alternatively, for GPU-accelerated perception (Isaac ROS / nvblox), run the
stack inside the Isaac ROS dev container — see [Isaac ROS Setup](docs/isaac_ros_setup.md).

### Cloning and Building

```bash
git clone --recurse-submodules https://github.com/UET-RE/UET_RE_AMR.git
cd UET_RE_AMR

# Automated setup (installs deps, builds)
./tools/setup_dev.sh   # or ./tools/setup_prod.sh on a robot/deployment machine
```

### Running Simulation

To launch the robot model in a Gazebo simulation environment:

```bash
ros2 launch uet_amr_simulation simulation.launch.py
```

### Running on Physical Hardware

To launch the robot control interface on physical hardware:

```bash
ros2 launch uet_amr_bringup amr_bringup.launch.py
```

---

## Documentation

| Document | Description |
|---|---|
| [Getting Started](docs/getting_started.md) | Installation and workspace configuration guide |
| [D435i Guide](docs/d435i_guide.md) | Setup and usage of the Intel RealSense D435i depth camera |
| [Jetson RealSense Setup](docs/jetson_realsense_setup.md) | Using RealSense on Jetson (source-built librealsense2) |
| [Isaac ROS Setup](docs/isaac_ros_setup.md) | Isaac ROS (nvblox) + GPU Docker dev workflow |
| [Navigation Package](ros2/src/uet_amr_navigation/README.md) | SLAM -> save map -> Nav2 workflow |
| [Hardware Docs](hardware/README.md) | Schematics, PCB layouts, CAD models, and BOM |
| [Hoverboard Controller Firmware](firmware/amr_hoverboard_controller/README.md) | STM32 motor control firmware |
| [UART Bridge Firmware](firmware/amr_uart_bridge/README.md) | ESP32 UART bridge for sensors/status |

---

## Team & Contribution Workflow

For details on branching, commit messages, and contribution guidelines, see [`CONTRIBUTING.md`](CONTRIBUTING.md)

## Acknowledgements

UET_RE_AMR builds on the work of several open-source projects and communities:

- **[linorobot2](https://github.com/linorobot/linorobot2)** — the overall system architecture and ROS2 integration approach for this project were inspired by linorobot2's design.
- **[hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)** by [EFeru](https://github.com/EFeru) — the base for `firmware/amr_hoverboard_controller`, adapted with a custom communication protocol and motor control logic for this platform. Licensed under GPL-3.0; see [`firmware/amr_hoverboard_controller/LICENSE`](firmware/amr_hoverboard_controller/LICENSE).
- **[rplidar_ros](https://github.com/Slamtec/rplidar_ros)** by Slamtec — vendored as a submodule at `ros2/src/third_party/rplidar_ros` for LiDAR driver support.
- **[Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS)** by NVIDIA — `nvblox_ros` provides GPU-accelerated 3D reconstruction feeding `uet_amr_navigation`'s Nav2 local costmap via `nvblox_nav2` (auto-selected when a GPU is present, CPU fallback otherwise); see [Isaac ROS Setup](docs/isaac_ros_setup.md).
- **[Nav2](https://github.com/ros-planning/navigation2)**, **[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)**, and **[robot_localization](https://github.com/cra-ros-pkg/robot_localization)** — the ROS2 navigation, SLAM, and sensor-fusion stacks this project is built on top of.


## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

Copyright (c) 2026 Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi.
