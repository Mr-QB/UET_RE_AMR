
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Docker-lightgrey)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

Autonomous Mobile Robot (AMR) project developed by the Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi.

---

## Overview

UET_RE_AMR is an autonomous mobile robot platform designed for research and educational purposes. The system integrates a ROS2-based software stack for navigation and high-level control with micro-ROS based firmware for low-level motor actuation, sensor feedback, and power management.

Key features include:
- Simultaneous Localization and Mapping (SLAM)
- Autonomous path planning and obstacle avoidance using Nav2
- Sensor integration including 2D LiDAR, IMU, and cameras
- Hardware interface using ros2_control for real-time differential drive control

---

## Repository Structure

```
UET_RE_AMR/
├── ros2/           # ROS2 workspace (navigation, perception, control)
├── firmware/       # Embedded firmware (STM32/ESP32)
├── hardware/       # Schematics, PCB layouts, CAD models
├── simulation/     # Gazebo simulation environments and models
├── docs/           # Technical documentation and guides
├── tools/          # System configuration and utility scripts
└── docker/         # Containerized development environments
```

For detailed software architecture details, refer to `docs/architecture.md`.

---

## Quick Start

### Prerequisites

To run the ROS2 stack natively, you need Ubuntu 22.04 and ROS2 Humble:

```bash
# Update package lists
sudo apt update

# Install core ROS2 and Nav2 dependencies
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim
```

Alternatively, you can run the stack using Docker (recommended):

```bash
docker compose -f docker/ros2/docker-compose.yml up
```

### Cloning and Building

```bash
git clone --recurse-submodules https://github.com/UET-RE/UET_RE_AMR.git
cd UET_RE_AMR

# Install ROS2 package dependencies
cd ros2
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

### Running Simulation

To launch the robot model in a Gazebo simulation environment:

```bash
ros2 launch uet_amr_bringup amr_simulation.launch.py
```

### Visualizing with RViz2

Open RViz2 to inspect the robot model, TF tree, LaserScan data, and map while the simulation is running:

```bash
# Terminal 1 - Start Gazebo (if not already running)
source install/setup.bash
ros2 launch uet_amr_bringup amr_simulation.launch.py

# Terminal 2 - Open RViz2
source install/setup.bash
ros2 launch uet_amr_simulation rviz.launch.py
```

Default displays in RViz2:
- **RobotModel** – 3D robot model visualization
- **TF** – transform tree `map → odom → base_footprint → base_link → ...`
- **LaserScan** – real-time LiDAR data (topic `/scan`)
- **Odometry** – velocity and position vectors (topic `/odom`)

### 2D Mapping (SLAM)

Run 2D mapping using `slam_toolbox`. Open **4 separate terminals**, each with the workspace sourced:

**Terminal 1 – Gazebo simulation:**
```bash
source install/setup.bash
ros2 launch uet_amr_bringup amr_simulation.launch.py
```

**Terminal 2 – SLAM Toolbox (map building):**
```bash
source install/setup.bash
ros2 launch uet_amr_navigation slam.launch.py
```

**Terminal 3 – Teleop (drive the robot):**
```bash
source install/setup.bash
ros2 launch uet_amr_teleop teleop.launch.py
```

**Terminal 4 – RViz2 (visualize the map):**
```bash
source install/setup.bash
ros2 launch uet_amr_simulation rviz.launch.py
```

In RViz2, add a **Map** display → topic `/map` to view the map being built in real-time.

**Save the map when mapping is complete:**
```bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This generates `my_map.pgm` and `my_map.yaml`, which can be used for autonomous navigation.

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
| [Architecture Guide](docs/architecture.md) | Overview of system architecture and packet flow |
| [Hardware Setup](docs/hardware_setup.md) | Pinouts, electrical connections, and assembly |
| [ROS2 Packages](docs/ros2/packages_overview.md) | Description of custom ROS2 packages and nodes |
| [Firmware Flashing](docs/firmware/flashing_guide.md) | Flashing firmware onto the microcontrollers |
| [Serial Protocol](docs/firmware/serial_protocol.md) | Low-level serial communication packet format |

---

## Team & Contribution Workflow

For details on branching, commit messages, and contribution guidelines, see `CONTRIBUTING.md`.

### Component Ownership

| Team | Main Directory | Working Branch |
|---|---|---|
| Navigation | `ros2/src/uet_amr_navigation/` | `feature/navigation-*` |
| Perception | `ros2/src/uet_amr_perception/` | `feature/perception-*` |
| Hardware Interface | `ros2/src/uet_amr_hardware/` | `feature/hardware-*` |
| Firmware | `firmware/` | `feature/firmware-*` |
| Simulation | `simulation/` | `feature/sim-*` |

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

Copyright (c) 2026 Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi.
