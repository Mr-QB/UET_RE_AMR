# Hardware Status Message
# Published by: uet_amr_hardware (AmrHardwareInterface)
# Topic: /hardware/status  (1 Hz, latest firmware feedback snapshot)
#
# No battery voltage field: the ESP32-to-ROS feedback packet (ros_protocol.h)
# only forwards a computed battery percentage, not the hoverboard mainboard's
# raw batVoltage reading. Add it here (and to the wire protocol on both ends)
# if raw voltage is ever needed.

std_msgs/Header header

# Firmware state (sys_status)
uint8 SYS_IDLE = 0
uint8 SYS_NAV = 1
uint8 SYS_MANUAL = 2
uint8 SYS_FAULT = 3
uint8 sys_status

# Battery / board
float32 battery_percentage   # State of charge [0.0 - 1.0]
string  battery_state        # "OK" | "LOW" | "CRITICAL"
bool    is_charging
float32 current              # Current draw [A]
float32 temperature          # Board temperature [degC]

# Drivetrain
int8    left_speed           # Measured wheel speed, firmware units [-100..100]
int8    right_speed
float64 left_wheel_position  # [rad]
float64 right_wheel_position
float64 left_wheel_velocity  # [rad/s]
float64 right_wheel_velocity
