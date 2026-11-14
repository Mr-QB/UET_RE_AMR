# Hardware Status Message
# Published by: uet_amr_hardware (AmrHardwareInterface)
# Topic: /hardware/status  (1 Hz, latest firmware feedback snapshot plus serial link health)
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

# Serial link health (ESP32 -> ROS feedback stream). "total" counts since the controller started;
# "window" values cover the last status period (about 1 s).
uint64  link_packets_total
uint32  link_frames_lost_total        # feedback frames missed, from gaps in the sequence number
uint32  link_checksum_errors_total    # packets that arrived complete but failed the checksum
uint32  link_bytes_discarded_total    # bytes skipped while looking for a packet start / dropped corrupt packets
uint32  link_timeouts_total           # >500 ms of silence, forced a port reopen
uint32  link_reconnects_total
float32 link_rate_hz                  # window: packets per second (the firmware sends about 49 Hz)
float32 link_loss_percent             # window: lost / (lost + received)
float32 link_firmware_period_ms       # window: mean packet spacing by the firmware's own clock
float32 link_delay_mean_ms            # window: arrival delay above the best case of the window
float32 link_delay_max_ms             # (delay VARIATION from queuing/scheduling, not the absolute latency)
