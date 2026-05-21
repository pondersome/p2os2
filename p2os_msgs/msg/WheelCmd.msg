# Independent per-wheel velocity command (tank / differential drive).
# Sent to ARCOS as the VEL2 command (32), which sets each wheel's
# velocity-loop setpoint directly — bypassing the VEL+RVEL translation/
# rotation mixing and the firmware rotation PID.
#
# Intended for bench diagnostics: drive one wheel at a time, or both at
# equal speed, to isolate a faulty motor / encoder / H-bridge channel
# without the controller redistributing effort and masking the fault.
#
# Requires the p2os_driver `enable_wheel_cmd` parameter to be true; with
# it false (the default) the driver does not subscribe to this topic at
# all. The wheels are zeroed on command silence (see `wheel_cmd_timeout`).
float32 left     # left wheel ground velocity, m/s, + = forward
float32 right    # right wheel ground velocity, m/s, + = forward
