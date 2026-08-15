# Getting Started

This guide provides installation and environment setup instructions for new developers joining the UET_RE_AMR project.

---

## 1. System Requirements

- **Operating System**: Ubuntu 22.04 LTS (native installation is recommended; WSL2 is supported for simulation only).
- **RAM**: Minimum 8 GB (16 GB or more is recommended when running Gazebo simulations).
- **Disk Space**: At least 20 GB of free space.

---

## 2. ROS2 Humble Installation

Follow these steps to install ROS2 Humble Desktop:

```bash
# Add the ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Desktop package
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-dev-tools

# Source the setup script
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Cloning and Setting Up the Workspace

```bash
git clone --recurse-submodules https://github.com/UET-RE/UET_RE_AMR.git
cd UET_RE_AMR

# Run the automated setup script
chmod +x tools/setup_dev.sh tools/setup_prod.sh

# For developers (adds PlatformIO + micro-ROS agent for firmware work)
./tools/setup_dev.sh

# For production / deployment machines (ROS2 workspace only)
./tools/setup_prod.sh
```

### Manual Installation Steps (Alternative)

If you prefer to install dependencies manually:

```bash
# Install Nav2, ros2_control, and SLAM packages
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-slam-toolbox \
  ros-humble-diff-drive-controller

# Install dependencies using rosdep
cd ros2
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

---

## 4. PlatformIO Installation (Firmware)

PlatformIO Core is used to manage libraries, build, and flash MCU firmware:

```bash
pip install platformio

# Verify installation
pio --version
```

---

## 5. Running the Simulation

Ensure your workspace is built and sourced before running:

```bash
cd ros2
source install/setup.bash

# Phase 1: SLAM mapping in sim (RViz launches automatically)
ros2 launch uet_amr_bringup amr_simulation.launch.py mode:=slam

# In a separate terminal: drive around with teleop to build the map
ros2 launch uet_amr_teleop keyboard_teleop.launch.py

# Once you've mapped the area, save it (see uet_amr_navigation/maps/README.md)
ros2 run nav2_map_server map_saver_cli -f src/uet_amr_navigation/maps/warehouse

# Phase 2: Nav2 navigation against the saved map
ros2 launch uet_amr_bringup amr_simulation.launch.py mode:=nav

# In RViz, use "2D Goal Pose" to send navigation goals.
```

---

## 6. Flashing Firmware (Hardware)

To upload the controller firmware to the microcontroller (MCU):

```bash
# Connect the MCU to your PC via USB
cd firmware/base_controller

# Edit the upload_port configuration in platformio.ini to match your device port
# Build and upload the firmware
pio run --target upload

# Start the serial monitor
pio device monitor
```

---

## 7. Running on the Robot Hardware

```bash
# Start the micro-ROS agent to bridge communications
./tools/start_microros_agent.sh

# Phase 1: SLAM mapping on real hardware (base + sensors + slam_toolbox + RViz)
ros2 launch uet_amr_bringup amr_hardware.launch.py mode:=slam serial_port:=/dev/ttyUSB0

# In a separate terminal: drive around with teleop to build the map
ros2 launch uet_amr_teleop keyboard_teleop.launch.py

# Once you've mapped the area, save it (see uet_amr_navigation/maps/README.md)
ros2 run nav2_map_server map_saver_cli -f src/uet_amr_navigation/maps/warehouse

# Phase 2: Nav2 navigation against the saved map
ros2 launch uet_amr_bringup amr_hardware.launch.py mode:=nav

# In RViz, use "2D Goal Pose" to send navigation goals.
```

---

## 8. Troubleshooting

### Build error: "package not found"
Ensure all required system packages are installed by running rosdep:
```bash
rosdep install --from-paths ros2/src --ignore-src -r -y
```

### Serial port permission denied
If you cannot connect to the MCU, add your user account to the dialout group:
```bash
# Check the assigned port
ls /dev/ttyUSB*
# Grant read/write permissions to the current user
sudo usermod -a -G dialout $USER
```
Reboot or log out and log back in for changes to take effect.

### Gazebo launch failures
Verify that the Gazebo (new, Ignition-based) packages are correctly installed and integrated with ROS2:
```bash
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ign-ros2-control
```
