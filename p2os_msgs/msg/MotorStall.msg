std_msgs/Header header
# Per-wheel stall flags from the ARCOS SIP (bit 0 of each bumper byte,
# independent of the bumper segments). The firmware sets these on a
# commanded-vs-actual wheel-velocity discrepancy — a true motor stall,
# e.g. an H-bridge thermal cutout or a mechanical stall.
bool left
bool right
