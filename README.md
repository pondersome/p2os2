# p2os2 - updated P2OS for ROS2 (Jazzy)

P2OS is a driver for the Pioneer robots. This driver was originally written for Player/Stage by Brian Gerkey.

This update to P2OS now runs in ROS2 (currently tracking Jazzy, previously Humble). There are unknowns since there are subsystems I do not have access to and therefore have not tested. These include anything to do with laser scanners, the PTZ camera (I converted the node but could not test it), the arm/gripper subsystem and the native wired joystick. All of the SIP packets and related ROS2 messages pertaining to these missing systems were preserved in the migration, but again I can't test them. My goal was to get the chassis moving and the sonars working. I only have access to a Pioneer 3AT, so could not test and did not migrate launch configurations for the P3DS. Also did not attempt to migrate the Player/Stage worlds since that project seems dormant, although there are some ROS2 migrations of Player/Stage that could be tried if anyone else wants to pursue that.


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

**Sonar fan-out (2026-04).** The driver now publishes three parallel
representations of the sonar array using the per-sensor geometry that
already lived in `robot_params.cpp` (previously never read). On any
Pioneer model with a populated `sonar_pose` table — 16 sensors for
the P3AT — the driver produces:

- `sonar` (`p2os_msgs/SonarArray`) — legacy topic, unchanged
- `sonar/range_<N>` (`sensor_msgs/Range`) — 16 per-sensor topics with
  proper `frame_id`, `field_of_view`, `min_range`, `max_range`. What
  Nav2's `nav2_costmap_2d::RangeSensorLayer` expects
- `sonar/cloud` (`sensor_msgs/PointCloud2`) — `xyz + sensor_id`, one
  point per live reading, projected in the parent frame. Drops
  no-echo sentinels so visualization stays clean

Plus 16 static TFs (`<namespace>/sonar_0 … sonar_15`) broadcast once
at startup with transient_local QoS. The `use_sonar` parameter is
dynamic — toggle at runtime with `ros2 param set`. Clean-shutdown
path (`SONAR 0` before `STOP`/`CLOSE`) ensures the next startup
syncs with ARCOS without requiring a chassis reset.

See [sonar_operation.md](../../ponderdocs/p2os2/docs/sonar_operation.md)
for config and usage; [sonar_architecture.md](../../ponderdocs/p2os2/docs/sonar_architecture.md)
for design decisions and rationale.



p2os_launch
-----------

Legacy ROS1 launch files for the Robot. Not usable in ROS2. Kept as reference of prior usage. Contains examples of Stage simulation worlds.


p2os_teleop
-----------

Control the robot with a joystick or keyboard. 


p2os_urdf
---------

Allows you to see the robot within RVIZ for navigation purposes. 

