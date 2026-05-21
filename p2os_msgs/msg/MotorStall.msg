std_msgs/Header header
# Per-wheel stall flags from the ARCOS SIP (bit 0 of each bumper byte,
# independent of the bumper segments). The firmware sets these on a
# commanded-vs-actual wheel-velocity discrepancy — a true motor stall,
# e.g. an H-bridge thermal cutout or a mechanical stall.
bool left
bool right
# Per-wheel measured velocity from the SIP (LVEL/RVEL, m/s). These are
# the encoder-derived wheel speeds the firmware compares against the
# command to set the stall flags above. Published per-wheel (the
# /pose Odometry only carries the averaged twist) so a controller-
# independent, open-loop per-wheel comparison is possible — the
# decisive signal for localizing an asymmetric drive fault.
float32 left_vel
float32 right_vel
