# UET AMR UART BRIDGE

UART bridge using an ESP32 as an intermediary device for data transmission without modifying the data. The ESP32 can also be extended with various peripheral devices such as an IMU, HC-SR04, etc., providing additional information to ROS 2.

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
| Obstacle detection    | HC-SR04(optional)               |
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

The Mega/ESP32 validates the received packet and converts it to the Hoverboard command format:

```cpp
typedef struct __attribute__((packed)) {
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
```

## Hoverboard -> ROS2

The Hoverboard sends the feedback packet to Mega/ESP32 via UART:

```cpp
typedef struct __attribute__((packed)) {
  uint16_t  start;
  int8_t    speedR_meas; // rpm left
  int8_t    speedL_meas; // rpm right
  int16_t   wheelR_cnt;  // encoder left
  int16_t   wheelL_cnt;  // encoder right
  uint8_t   batVoltage;
  uint8_t   boardTemp;
  int16_t   cur;         // current draw
  uint16_t  checksum;
} SerialFeedback;
```

The Mega/ESP32 validates the received packet and package all information in this format:

(Data optimization for high-speed transmission).

16-Byte Bit Map Layout (128 Bits Total).
| Word | Bit Range | Field Name | Size | Type | Range / Function |
| :---: | :---: | :--- | :---: | :--- | :--- |
| **W1** | **0 – 7** | `header` | 8 bit | `uint8_t` | Sync header (`0xAA`) |
| **W1** | **8 – 9** | `sys_status` | 2 bit | Unsigned Int | Mode: `0`=Idle, `1`=Nav, `2`=Manual, `3`=Fault |
| **W1** | **10 – 15** | `reserved1` | 6 bit | Alignment | Spare padding bits |
| **W1** | **16 – 23** | `speed_l` | 8 bit | Signed Int | Left motor speed (-100 to +100 %, clamped) |
| **W1** | **24 – 31** | `speed_r` | 8 bit | Signed Int | Right motor speed (-100 to +100 %, clamped) |
| **W2** | **32 – 45** | `en_tick_l` | 14 bit | Unsigned Int | Left encoder tick counter (0–9000) |
| **W2** | **46 – 59** | `en_tick_r` | 14 bit | Unsigned Int | Right encoder tick counter (0–9000) |
| **W2** | **60 – 63** | `reserved2` | 4 bit | Alignment | Spare padding bits |
| **W3** | **64 – 70** | `temp_c` | 7 bit | Unsigned Int | Temperature 0°C to 127°C integer |
| **W3** | **71 – 81** | `current_a` | 11 bit | Unsigned Int | Current (0.00 to 20.47 A, **divide by 100 on read**) |
| **W3** | **82 – 88** | `battery` | 7 bit | Unsigned Int | Battery percentage (0–100%) |
| **W3** | **89** | `charging` | 1 bit | Boolean | Charger state: `0`=Discharging, `1`=Charging |
| **W3** | **90 – 95** | `optional_1` | 6 bit | Raw Bits | Custom payload expansion 1 |
| **W4** | **96 – 111** | `optional_2` | 16 bit | Raw Bits | Custom payload expansion 2 |
| **W4** | **112 – 127** | `crc` | 16 bit | CRC-16 | Checksum calculated over bits 0–111 |

---

C++ Struct Definition

```cpp
typedef struct __attribute__((packed)) {
    // Word 1 (Bits 0..31): Header, Mode, Alignment, Signed Speeds (-100..100)
    uint32_t header      : 8;  // Bits 0..7   (Sync byte 0xAA)
    uint32_t sys_status  : 2;  // Bits 8..9   (0=Idle, 1=Nav, 2=Manual, 3=Fault)
    uint32_t reserved1   : 6;  // Bits 10..15 (Spare alignment bits)
    int32_t  speed_l     : 8;  // Bits 16..23 (Clamped -100 to +100)
    int32_t  speed_r     : 8;  // Bits 24..31 (Clamped -100 to +100)

    // Word 2 (Bits 32..63): Wheel Encoders & Spare Bits
    uint32_t en_tick_l   : 14; // Bits 32..45 (0-9000 ticks)
    uint32_t en_tick_r   : 14; // Bits 46..59 (0-9000 ticks)
    uint32_t reserved2   : 4;  // Bits 60..63 (Spare alignment bits)

    // Word 3 (Bits 64..95): 7-bit Temp (0..127°C), 11-bit Current, Battery & Power
    uint32_t temp_c      : 7;  // Bits 64..70 (0-127°C direct integer)
    uint32_t current_a   : 11; // Bits 71..81 (0-2047 raw value, divide by 100 on read -> 0.00 to 20.47 A)
    uint32_t battery     : 7;  // Bits 82..88 (0-100%)
    uint32_t charging    : 1;  // Bit 89      (0=Discharging, 1=Charging)
    uint32_t optional_1  : 6;  // Bits 90..95 (Expansion payload 1)

    // Word 4 (Bits 96..127): Optional Expansion 2 & CRC Checksum
    uint32_t optional_2  : 16; // Bits 96..111 (Expansion payload 2)
    uint32_t crc         : 16; // Bits 112..127 (CRC-16 Checksum)
} SlaveToMasterPacket16;                             // EXACTLY 16 Bytes (128 Bits)

static_assert(sizeof(SlaveToMasterPacket16) == 16, "Packet size must be 16 bytes!");
```

The remaining bits can be used to store additional data.

---

## Peripheral devices

HC-SR04 : 8pcs

position of HC-SR04:

        ----1----2----
        |            |
        8      ^     3
        |      |     |
        7            4
        |            |
        ----6----5----

Trig pin:

- 1 and 5 share the same pin
- 2 and 6 share the same pin
- 3 and 7 share the same pin
- 4 and 8 share the same pin

Echo pin: One pin per device(use interrupt)

12 pin total

Timing budget per device: 5ms (range: 3-80cm)

2 device run concurrently(e.g., 1&5, 2&6, 3&7, 4&8)

...
