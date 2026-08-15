# Battery Status Message
# Published by: uet_amr_hardware (AmrHardwareInterface)
# Topic: /battery/status
#
# No voltage field: the ESP32-to-ROS feedback packet (ros_protocol.h) only
# forwards a computed battery percentage, not the hoverboard mainboard's raw
# batVoltage reading (see hover_protocol.h). Add it here (and to the wire
# protocol on both ends) if raw voltage is ever needed.

std_msgs/Header header

float32 current      # Current draw [A]
float32 percentage   # State of charge [0.0 - 1.0]
float32 temperature  # Board temperature [degC]
bool    is_charging
string  status        # "OK" | "LOW" | "CRITICAL"
