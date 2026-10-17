# Motion-test executor state.
# Published by: amr_mode_manager (MotionTestExecutor)
# Topic: /amr/motion_test/state  (20 Hz; the terminal state keeps repeating until the next start)

# stamp = command-generation ROS time; frame_id = "odom"
std_msgs/Header header
uint64 sequence
string run_id
# idle | accepted | running | settling | completed | aborted | failed
string phase
# Why the run ended (terminal phases) or why the robot is not ready (idle)
string reason
# Robot-confirmed: slam | nav | stopped
string mode
# none | test | joy | keyboard
string manual_source
bool ready
# linear | rotation | linear_velocity | angular_velocity (empty when idle)
string kind
# Target, or target_velocity for velocity steps
float64 target
# Signed distance [m] or unwrapped angle [rad] since the start pose
float64 progress
# linear: perpendicular offset; rotation: distance from the start position
float64 lateral_error
# linear: yaw - initial yaw; rotation: target - progress
float64 heading_error
# Actual published velocity setpoints (NOT the estimate)
float64 commanded_linear
float64 commanded_angular
# From odometry twist
float64 measured_linear
float64 measured_angular
# Source odometry header stamp, never replaced by the current time
builtin_interfaces/Time measurement_stamp
