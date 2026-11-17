# Hardware Documentation

This directory contains the electrical schematics, electrical connections, PCB layouts, and mechanical CAD designs for the UET AMR platform.

---

## Directory Structure

```
hardware/
├── udev/            # udev rules: /dev/esp32, /dev/rplidar, RealSense USB access (installed by scripts/amr setup)
└── BOM.xlsx         # Project Bill of Materials
```

---

## Bill of Materials

Refer to `BOM.xlsx` for a complete list of mechanical parts, fasteners, electronics, sensors, and purchasing links.

---

## Controllers

The base uses two microcontrollers (firmware in `firmware/`):

| Sub-system | Microcontroller | Primary Function | Source Directory |
|---|---|---|---|
| Motor controller | STM32F103RC | Hoverboard motor control, wheel ticks, battery/temperature feedback | `firmware/amr_hoverboard_controller` |
| UART bridge | ESP32 | Bridges the STM32 serial link to the ROS 2 hardware interface (Protocol v2) | `firmware/amr_uart_bridge` |

---
## Power System

The robot is powered by a rechargeable battery pack.

### Power Distribution

| Device | Voltage | Current(max) |
|---|---|---:|
| Motor, Motor Driver | 36V | 10 A |
| Jetson | 12V | 3.4 A |
| D435i| 5V | 0.5 A |
| RPLidar | 5V | 1.5  A |
| ESP32 | 5V | 0.5  A |
| LED | 5V | 0.1  A |

---

### udev rules

`udev/` has the host udev rules. `scripts/amr setup` installs them.

| Device | Name | Rule |
|---|---|---|
| ESP32 / STM32 (CH340/CH341/CH9102) | `/dev/esp32` | `99-esp32.rules` |
| RPLidar | `/dev/rplidar` | `99-rplidar.rules` |
| RealSense D435/D435i/D455 | USB access | `99-realsense-libusb.rules` |

The launch files use `serial_port:=/dev/esp32` by default; override with `serial_port:=/dev/ttyUSBx`.
Credits are in `udev/NOTICE`.
