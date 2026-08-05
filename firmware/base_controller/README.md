# UET AMR Base Controller Firmware

Low-level base controller firmware for the UET AMR differential drive robot. This firmware runs on an Arduino Mega or ESP32 and communicates with a ROS2 node via a serial interface.

---

## Features
- Three-phase encoder reading and wheel odometry calculation
- Publishes wheel odometry information back to the ROS2 workspace
- Monitors battery status and motor diagnostics

---

## Hardware Requirements

| Component             | Recommendation                                       |
| --------------------- | ---------------------------------------------------- |
| Microcontroller (MCU) | Arduino Mega or ESP32-WROOM-32D                      |
| Motor Driver          | HoverBoard STM32F103RCTL, Brushless Motor Controller |
| Control Driver Type   | Hardware - UART                                      |
| FeedBack              | Hardware - UART                                      |
| Encoder               | Three-phase magnetic encoder                         |
| Communications        | High-speed UART interface (via USB-to-UART bridge)   |

---

## Build and Flash Instructions

Ensure PlatformIO Core is installed on your development machine before proceeding.

### Configuration (platformio.ini)

Modify the `platformio.ini` configuration file to match your hardware:

- `platform`: Specify the development platform/toolchain for your target MCU (e.g., `espressif32`, `ststm32`).
- `board`: Update to your targeted MCU board (e.g., `nucleo_f446re`, `esp32dev`).
- `framework`: Specify the development framework to use (e.g., `arduino`, `espidf`).
- `upload_port`: Specify the serial port assigned to the connected MCU (e.g., `COM9`, `/dev/ttyUSB0`).

---


## Setup Configurations

Update the GPIO mappings in `src/main.cpp` to match your custom controller board layout:

```cpp
//UART      Baudrate
Serial.begin(921600); // communicate with ROS2 

// UART          Buadrate             TX   RX
HoverSerial.begin(921600, SERIAL_8N1, 16, 17); // communicate with Hoverboard
```
