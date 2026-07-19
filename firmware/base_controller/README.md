# UET AMR Base Controller Firmware

Low-level base controller firmware for the UET AMR differential drive robot. This firmware runs on an Arduino Nano, Arduino Mega or ESP32 and communicates with a ROS2 node via a serial interface.

---

## Features

- Subscribes to `/cmd_vel` velocity commands (geometry_msgs/Twist) from the ROS2 host
- Closed-loop PID velocity control for two driving motors
- Quadrature encoder reading and wheel odometry calculation
- Publishes wheel odometry information back to the ROS2 workspace
- Monitors battery status and motor diagnostics

---

## Hardware Requirements

| Component             | Recommendation                                                                            |
| --------------------- | ----------------------------------------------------------------------------------------- |
| Microcontroller (MCU) | Arduino Nano, Arduino Mega or ESP32-WROOM-32D                                             |
| Motor Driver          | TB6612FNG, Cytron MD10C, or ODrive (depending on motor power), Brushless Motor Controller |
| Encoder               | Three-phase magnetic encoder                                                              |
| Communications        | High-speed UART interface (via USB-to-UART bridge)                                        |

---

## Build and Flash Instructions

Ensure PlatformIO Core is installed on your development machine before proceeding.

### PlatformIO Workflow

```bash
# Verify PlatformIO is installed
pio --version

# Build the firmware
cd firmware/base_controller
pio run

# Upload the binary to the connected microcontroller
pio run --target upload

# Start the serial monitor
pio device monitor
```

### Configuration (platformio.ini)

Modify the `platformio.ini` configuration file to match your hardware:

- `board`: Update to your targeted MCU board (e.g., `nucleo_f446re`, `esp32dev`).
- `upload_port`: Specify the serial port assigned to the connected MCU (e.g., `COM3`, `/dev/ttyUSB0`).

---

## micro-ROS Communication

The firmware uses micro-ROS client libraries to communicate directly with the high-level ROS2 stack.

### Starting the micro-ROS Agent on the Host

You must run a micro-ROS agent on the host computer to bridge communications:

```bash
# Option 1: Run via Docker (recommended)
docker run -it --rm --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200

# Option 2: Run natively
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
```

---

## PID Tuning

PID coefficients can be adjusted in the `include/controlMotor.h` header file:

```cpp
double kp = 1.5;   // Proportional gain (adjusts responsiveness)
double ki = 0.8;   // Integral gain (eliminates steady-state error)
double kd = 0.05;  // Derivative gain (reduces overshoot)
```

Actual wheel RPM can be monitored on the `/motors/status` topic.

---

## Pin Configurations

Update the GPIO mappings in `src/main.cpp` to match your custom controller board layout:

```cpp
// Left Motor Control Pins
#define LEFT_PWM_PIN   PA8
#define LEFT_DIR_PIN1  PA9
#define LEFT_DIR_PIN2  PA10

// Right Motor Control Pins
#define RIGHT_PWM_PIN  PB6
#define RIGHT_DIR_PIN1 PB7
#define RIGHT_DIR_PIN2 PB8

// Left Encoder Pins
#define LEFT_ENC_A     2
#define LEFT_ENC_B     3
#define LEFT_ENC_C     4

// Right Encoder Pins
#define RIGHT_ENC_A    5
#define RIGHT_ENC_B    6
#define RIGHT_ENC_C    7
```
