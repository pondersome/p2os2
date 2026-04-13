# p2os2 - updated P2OS for ROS2 Humble

P2OS is a driver for the Pioneer robots. This driver was originally written for Player/Stage by Brian Gerkey.

This update to P2OS now runs in ROS2. There are unknowns since there are subsystems I do not have access to and therefore have not tested. These include anything to do with laser scanners, the PTZ camera (I converted the node but could not test it), the arm/gripper subsystem and the native wired joystick. All of the SIP packets and related ROS2 messages pertaining to these missing systems were preserved in the migration, but again I can't test them. My goal was to get the chassis moving and the sonars working. I only have access to a Pioneer 3AT, so could not test and did not migrate launch configurations for the P3DS. Also did not attempt to migrate the Player/Stage worlds since that project seems dormant, although there are some ROS2 migrations of Player/Stage that could be tried if anyone else wants to pursue that.


p2os_bringup
------------

Launch configurations that were migrated to ROS2 python launch files for functionality I could test.

p2os_driver
-----------

Essential to P2OS is the driver. This controls the interface for the P2OS ARCOS controller.

**Breaking change (2026-04): cmd_vel silence watchdog is now active by default.**
The driver enforces a 200ms timeout on cmd_vel by default — if no new
cmd_vel arrives within that window and the last commanded velocity is
non-zero, the driver zeroes the wheels. This replaces the legacy
"send-once, robot moves until the next command" behavior. All continuous
publishers (Nav2 controllers, p2os_teleop, joystick teleop) satisfy this
contract out of the box. To restore the legacy behavior set
`cmd_vel_timeout:=0.0` on the driver. See
[p2os_velocity_and_acceleration.md](../../ponderdocs/p2os2/docs/p2os_velocity_and_acceleration.md#cmd_vel-silence-watchdog)
for the full discussion.



p2os_launch
-----------

Legacy ROS1 launch files for the Robot. Not usable in ROS2. Kept as reference of prior usage. Contains examples of Stage simulation worlds.


p2os_teleop
-----------

Control the robot with a joystick or keyboard. 


p2os_urdf
---------

Allows you to see the robot within RVIZ for navigation purposes. 

