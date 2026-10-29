# UET AMR UART BRIDGE

UART bridge using an ESP32 as an intermediary device for data transmission without modifying the data. The ESP32 can also be extended with various peripheral devices such as an IMU, etc., providing additional information to ROS 2.

---

## Features

- High-speed UART.
- Obstacle detection.
- Monitor system, motor, battery status.
- Safe control.

---

## Hardware Requirements

| Component             | Recommendation                  |
| --------------------- | ------------------------------- |
| Microcontroller (MCU) | Arduino Mega or ESP32-WROOM-32D |
| Status indication     | LED                             |
| ...                   | ...                             |

---

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

---

## Communication Protocol Specification: ROS2 <-> ESP32/Arduino Mega <-> Hoverboard(STM32)

Defines the communication interfaces and serial packet formats used to connect the low-level microcontroller (MCU) firmware with the high-level ROS2 software stack.

---

## Overview

The UET AMR platform uses UART as its primary communication interface, supporting bidirectional asynchronous serial communication between system components.

Arduino Mega/ESP32 serves as a high-speed, bidirectional UART bridge between ROS 2 and Hoverboard.

```
+------------------+         +------------------+         +------------------+
|    ROS2 Host     |  UART   |    Mega/ESP32    |  UART   |    Hoverboard    |
|   (Ubuntu PC)    |<------->|   (USB-Serial)   |<------->|     (STM32)      |
+------------------+         +------------------+         +------------------+
```

Baudrate: 921600 bit/s

The hoverboard continuously provides feedback at 200 Hz.

## Frequency
| Task      | Freq      |
|-------|--------|
|Control update| 50 Hz|
|Feedback update| 200 Hz|
|Obstacle detection| 50Hz|

*Control and feedback are independent of each other.


## ROS2 -> Hoverboard

ROS 2 sends the command packet to the Mega/ESP32 via UART:

```cpp
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint16_t left;
    uint16_t right;
    uint8_t checksum;
} SerialCommand;
```

The ESP32 validates the received hoverboard frame and forwards it to ROS2 as a 16-byte (128-bit) bit-packed frame
(see `include/ros_protocol.h`). The 16 bytes are one little-endian 128-bit value: bit 0 is the LSB of byte 0, and the
fields follow each other in this order with no padding.

| Bits | Field | Size | Meaning |
| :---: | :--- | :---: | :--- |
| 0 – 7 | `header` | 8 | Sync header `0xAA` |
| 8 – 14 | `seq` | 7 | Wraps at 128; a gap means lost packets |
| 15 | `estop` | 1 | `0`=released, `1`=e-stop active |
| 16 – 31 | `speed_l` | 16 | Left measured speed, signed, raw hoverboard rpm |
| 32 – 47 | `speed_r` | 16 | Right measured speed, signed, raw hoverboard rpm |
| 48 – 61 | `en_tick_l` | 14 | Left encoder ticks (0–9000, wraps) |
| 62 – 75 | `en_tick_r` | 14 | Right encoder ticks (0–9000, wraps) |
| 76 – 82 | `temp_c` | 7 | Board temperature, 0–127 °C |
| 83 – 93 | `current_a` | 11 | Raw; **divide by 100** for 0.00–20.47 A |
| 94 – 100 | `battery` | 7 | Battery percentage, 0–100 % |
| 101 – 111 | `stamp` | 11 | Sample time, 256 µs units, wraps every ~524 ms (host uses the difference for dt) |
| 112 – 127 | `checksum` | 16 | 16-bit XOR fold over bytes 0–13 |

Command, ROS -> bridge (8 bytes, byte-aligned):

| Byte | Field | Type | Meaning |
| :---: | :--- | :---: | :--- |
| 0 | `header` | u8 | `0xAA` |
| 1 – 2 | `left` | i16 LE | Commanded left speed, clamped to ±1000 |
| 3 – 4 | `right` | i16 LE | Commanded right speed, clamped to ±1000 |
| 5 – 6 | `reserved` | u16 | Unused (0) |
| 7 | `checksum` | u8 | XOR of bytes 0–6 |
