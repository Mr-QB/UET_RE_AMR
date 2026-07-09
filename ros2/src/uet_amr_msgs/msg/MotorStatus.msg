# Motor Status Message
# Published by: uet_amr_hardware
# Topic: /motors/status

std_msgs/Header header

float32 left_rpm       # Left motor RPM
float32 right_rpm      # Right motor RPM
float32 left_current   # Left motor current [A]
float32 right_current  # Right motor current [A]
bool    left_fault     # Motor fault flag
bool    right_fault    # Motor fault flag
