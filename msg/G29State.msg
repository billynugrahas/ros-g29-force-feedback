# G29 Wheel State Message
# Provides real-time feedback about the wheel's current state

std_msgs/Header header

# Position information (normalized -1.0 to 1.0)
# Multiply by 450 to get degrees (-450° to +450°)
float32 current_position    # Current wheel position
float32 target_position     # Target position from last command

# Force information (normalized 0.0 to 1.0)
# Multiply by 2.5 to get approximate Nm
float32 applied_torque      # Currently applied torque magnitude

# Operating mode
string mode                 # Current control mode: "idle", "rotating", "braking", "centering"

# System status
bool device_connected       # Device handle is valid and operational