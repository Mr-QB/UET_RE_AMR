# UET AMR Hoverboard Controller Firmware

Low-level base controller firmware for the UET AMR differential drive robot. This firmware runs on an Hoverboard STM32F103RCTL and communicates with a ROS2 node via a UART bridge.

This project is based on the original the hoverboard-firmware-hack project:
- Original project: [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC.git)
- Original author: EFeru
- Licensed under the GNU General Public License v3.0 (GPL-3.0). See the [`LICENSE`](LICENSE) file for the full license text.

This version contains modifications made for the project, including changes to the communication protocol and motor control.

## Features

- Three-phase encoder reading for wheel odometry calculation.
- Stable speed(RPM) by using PID control.
- Monitors battery, temperature, current draw status and motor diagnostics.
- Watchdog detect, auto power-off in long time no contact.
- The buzzer indicates the system status through audible signals.

## Hardware Requirements

| Component             | Recommendation                                     |
| --------------------- | -------------------------------------------------- |
| Microcontroller (MCU) | STM32F103RCTL                                      |
| Motor Driver          | HoverBoard                                         |
| Control Driver Type   | Hardware - UART                                    |
| FeedBack              | Hardware - UART                                    |
| Encoder               | Three-phase magnetic encoder                       |
| Communications        | High-speed UART interface (via USB-to-UART bridge) |

## Build and Flash Instructions

Ensure PlatformIO Core is installed on your development machine before proceeding.

### Configuration (platformio.ini)

Modify the `platformio.ini` configuration file to match your hardware:

```cpp
//uncomment this line to use UART
default_envs = VARIANT_USART
```

---

## Configurations

Update the GPIO mappings in `Inc/config.h` to match your custom controller board layout:

```cpp
#define ENABLE_ODOMETRY

#define CTRL_TYP_SEL    FOC_CTRL
#define CTRL_MOD_REQ    SPD_MODE  // speed mode

#define FEEDBACK_SERIAL_USART2    // use serial 2 to control and feedback
#define CONTROL_SERIAL_USART2  0

#define USART2_BAUD    921600     // set high baudrate for fast
```
