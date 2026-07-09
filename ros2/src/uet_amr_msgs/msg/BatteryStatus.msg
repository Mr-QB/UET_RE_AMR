# Battery Status Message
# Published by: firmware/base_controller → ROS2 bridge
# Topic: /battery/status

std_msgs/Header header

float32 voltage          # Battery voltage [V]
float32 current          # Current draw [A]
float32 percentage       # State of charge [0.0 - 1.0]
bool    is_charging
string  status           # "OK" | "LOW" | "CRITICAL"
