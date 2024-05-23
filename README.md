# p2os2 - updated P2OS for ROS2 Foxy

P2OS is a driver for the Pioneer robots. This driver was originally written for Player/Stage by Brian Gerkey.

This update to P2OS now runs in ROS2. There are unknowns since there are subsystems I do not have access to and therefore have not tested. These include anything to do with laser scanners, the PTZ camera (I converted the node but could not test it), the arm/gripper subsystem and the native wired joystick. All of the SIP packets and related ROS2 messages pertaining to these missing systems were preserved in the migration, but again I can't test them. My goal was to get the chassis moving and the sonars working. I only have access to a Pioneer 3AT, so could not test and did not migrate launch configurations for the P3DS. Also did not attempt to migrate the Player/Stage worlds since that project seems dormant, although there are some ROS2 migrations of Player/Stage that could be tried if anyone else wants to pursue that.


p2os_bringup
------------

Launch configurations that were migrated to ROS2 python launch files for functionality I could test.

p2os_driver
-----------

Essential to P2OS is the driver. This controls the interface for the P2OS ARCOS controller. 


p2os_launch
-----------

Legacy ROS1 launch files for the Robot. Not usable in ROS2. Kept as reference of prior usage. Contains examples of Stage simulation worlds.


p2os_teleop
-----------

Control the robot with a joystick or keyboard. 


p2os_urdf
---------

Allows you to see the robot within RVIZ for navigation purposes. 

