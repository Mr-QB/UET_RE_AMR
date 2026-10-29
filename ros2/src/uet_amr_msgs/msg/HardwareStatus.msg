# Hardware Status Message
# Published by: uet_amr_hardware (AmrHardwareInterface)
# Topic: /hardware/status  (1 Hz, latest firmware feedback snapshot)
#
# No battery voltage field: the ESP32-to-ROS feedback packet (ros_protocol.h)
# only forwards a computed battery percentage, not the hoverboard mainboard's
# raw batVoltage reading. Add it here (and to the wire protocol on both ends)
# if raw voltage is ever needed.

std_msgs/Header header

# Safety
bool    estop                # Emergency stop active

# Battery / board
float32 battery_percentage   # State of charge [0.0 - 1.0]
string  battery_state        # "OK" | "LOW" | "CRITICAL"
float32 current              # Current draw [A]
float32 temperature          # Board temperature [degC]

# Drivetrain
int16   left_speed           # Measured wheel speed, raw hoverboard rpm (unscaled)
int16   right_speed
float64 left_wheel_position  # [rad]
float64 right_wheel_position
float64 left_wheel_velocity  # [rad/s]
float64 right_wheel_velocity
