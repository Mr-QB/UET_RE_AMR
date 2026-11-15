# Serial Link Status Message
# Published by: uet_amr_hardware (AmrHardwareInterface)
# Topic: /hardware/link_status  (1 Hz, health of the ESP32 -> ROS feedback stream)

std_msgs/Header header

# "total" counts since the controller started;
# "window" values cover the last status period (about 1 s).
uint64  packets_total
uint32  frames_lost_total        # feedback frames missed, from gaps in the sequence number
uint32  checksum_errors_total    # packets that arrived complete but failed the checksum
uint32  bytes_discarded_total    # bytes skipped while looking for a packet start / dropped corrupt packets
uint32  timeouts_total           # >500 ms of silence, forced a port reopen
uint32  reconnects_total
float32 rate_hz                  # window: packets per second (the firmware sends about 49 Hz)
float32 loss_percent             # window: lost / (lost + received)
float32 firmware_period_ms       # window: mean packet spacing by the firmware's own clock
float32 delay_mean_ms            # window: arrival delay above the best case of the window
float32 delay_max_ms             # (delay VARIATION from queuing/scheduling, not the absolute latency)
