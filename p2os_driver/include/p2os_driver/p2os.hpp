/*
 *  P2OS for ROS
 *  Copyright (C) 2009  David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *      Richard Vaughan, & Andrew Howard
 *  Copyright (C) 2018  Hunter L. Allen
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef P2OS_DRIVER__P2OS_HPP_
#define P2OS_DRIVER__P2OS_HPP_
#include <pthread.h>
#include <sys/time.h>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <p2os_msgs/msg/motor_state.hpp>
#include <p2os_msgs/msg/gripper_state.hpp>
#include <p2os_msgs/msg/sonar_array.hpp>
#include <p2os_msgs/msg/dio.hpp>
#include <p2os_msgs/msg/aio.hpp>
#include <p2os_msgs/msg/battery_state.hpp>
#include <vector>

//#include <diagnostic_updater/publisher.h>
//#include <diagnostic_updater/diagnostic_updater.h>

#include <p2os_driver/packet.hpp>
#include <p2os_driver/robot_params.hpp>

#include <iostream>
#include <cstring>
#include <string>

/*! \brief Container struct.
 *
 *  Create a struct that holds the
 *  Robot's sensors.
 */
typedef struct ros_p2os_data
{
  //! Provides the position of the robot
  nav_msgs::msg::Odometry position;
  //! Provides the battery voltage
  p2os_msgs::msg::BatteryState batt;
  //p2os_msgs::msg::BatteryState batt;
  //! Provides the state of the motors (enabled or disabled)
  p2os_msgs::msg::MotorState motors;
  //! Provides the state of the gripper
  p2os_msgs::msg::GripperState gripper;
  //! Container for sonar data
  p2os_msgs::msg::SonarArray sonar;
  //! Digital In/Out
  p2os_msgs::msg::DIO dio;
  //! Analog In/Out
  p2os_msgs::msg::AIO aio;
  //! Transformed odometry frame.
  geometry_msgs::msg::TransformStamped odom_trans;
} ros_p2os_data_t;

// this is here because we need the above typedef's before including it.
#include "sip.hpp"
#include "kinecalc.hpp"

#include "p2os_ptz.hpp"

class SIP;

// Forward declaration of the KineCalc_Base class declared in kinecalc_base.h

class P2OSNode : public rclcpp::Node
{
  /*! \brief P2OS robot driver node.
   *
   *  This class contains, essentially, the main means of communication
   *  between the robot and ROS.
   */

public:
  explicit P2OSNode(const std::string & node_name);
  virtual ~P2OSNode();

public:
  //! Setup the robot for use. Communicates with the robot directly.
  int Setup();
  //! Prepare for shutdown.
  int Shutdown();
  rclcpp::Time get_current_time();

  int SendReceive(P2OSPacket * pkt, bool publish_data = true);

  void updateDiagnostics();

  void ResetRawPositions();
  void ToggleSonarPower(unsigned char val);
  void ToggleMotorPower(unsigned char val);
  void StandardSIPPutData(rclcpp::Time ts);

  inline double TicksToDegrees(int joint, unsigned char ticks);
  inline unsigned char DegreesToTicks(int joint, double degrees);
  inline double TicksToRadians(int joint, unsigned char ticks);
  inline unsigned char RadiansToTicks(int joint, double rads);
  inline double RadsPerSectoSecsPerTick(int joint, double speed);
  inline double SecsPerTicktoRadsPerSec(int joint, double secs);

  void SendPulse(void);

  //! Lazy one-shot. Uses the live PlayerRobotParams[param_idx] sonar_pose
  //! table to build per-sensor frames, cache conversions, create the
  //! 16 Range + 1 PointCloud2 publishers, and broadcast the static TFs.
  //! Safe to call from StandardSIPPutData — it requires param_idx, which
  //! is assigned during Setup().
  void init_sonar_extras();

  //! Fan out the current SonarArray into per-sensor Range messages and a
  //! single PointCloud2 (xyz + sensor_id). No-ops if the corresponding
  //! publish flags are false. Silently defers until init_sonar_extras()
  //! has run.
  void publish_sonar_extras(const rclcpp::Time & ts);

  //! Parameter callback: makes `use_sonar` runtime-toggleable. When it
  //! transitions it commands ARCOS (ToggleSonarPower 0|1) and updates
  //! the runtime publish gate. Other params are accepted without side
  //! effects; changing `sonar_frame_prefix`, `sonar_parent_frame`, or
  //! `sonar_z_offset_m` after startup doesn't re-emit the static TFs
  //! — toggling use_sonar is the supported dynamic knob.
  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params);

  void check_and_set_vel();
  //void cmdvel_cb(const geometry_msgs::msg::Twist::ConstSharedPtr &);
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void check_and_set_motor_state();
  //void cmdmotor_state(const p2os_msgs::MotorStateConstPtr &);
  void cmdmotor_state_callback(const p2os_msgs::msg::MotorState::SharedPtr msg);

  void check_and_set_gripper_state();
  ///void gripperCallback(const p2os_msgs::msg::GripperStateConstPtr & msg);
  void gripper_callback(const p2os_msgs::msg::GripperState::SharedPtr msg);

  double get_pulse() {return pulse;}
  

  // diagnostic messages
  //void check_voltage(diagnostic_updater::DiagnosticStatusWrapper & stat);
  //void check_stall(diagnostic_updater::DiagnosticStatusWrapper & stat);

protected:
  //todo remove after conversion - 6 lines
  //! Node Handler used for publication of data.
  //rclcpp::Node n;
  //! Node Handler used for private data publication.
  //rclcpp::Node nh_private;
  //diagnostic_updater::Updater diagnostic_;
  //diagnostic_updater::DiagnosedPublisher<p2os_msgs::BatteryState> batt_pub_;
  
/*  ros::Publisher mstate_pub_, grip_state_pub_,
    ptz_state_pub_, sonar_pub_, aio_pub_, dio_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber cmdvel_sub_, cmdmstate_sub_, gripper_sub_, ptz_cmd_sub_;
  */
  rclcpp::Publisher<p2os_msgs::msg::BatteryState>::SharedPtr batt_pub_;
  rclcpp::Publisher<p2os_msgs::msg::MotorState>::SharedPtr mstate_pub_;
  rclcpp::Publisher<p2os_msgs::msg::GripperState>::SharedPtr grip_state_pub_;
  rclcpp::Publisher<p2os_msgs::msg::PTZState>::SharedPtr ptz_state_pub_;
  rclcpp::Publisher<p2os_msgs::msg::SonarArray>::SharedPtr sonar_pub_;
  rclcpp::Publisher<p2os_msgs::msg::AIO>::SharedPtr aio_pub_;
  rclcpp::Publisher<p2os_msgs::msg::DIO>::SharedPtr dio_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;

  //! Per-sensor Range publishers. Sized to the active robot's SonarNum
  //! at init_sonar_extras() time (16 for a P3AT).
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> sonar_range_pubs_;
  //! Single PointCloud2 topic with one point per live sonar reading.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_cloud_pub_;
  //! Emits the per-sensor frames once at init time. Must outlive the
  //! node so late subscribers still receive them via transient_local QoS.
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> sonar_static_broadcaster_;
  //! Cached per-sensor frame ids, matching array index.
  std::vector<std::string> sonar_frame_ids_;
  //! Cached per-sensor mount (x, y) in meters, base-link frame.
  std::vector<std::pair<double, double>> sonar_xy_m_;
  //! Cached per-sensor mount yaw in radians, base-link frame.
  std::vector<double> sonar_theta_rad_;
  //! True once init_sonar_extras() has successfully configured itself.
  bool sonar_extras_ready_ = false;
  //! Handle must be retained for the parameter callback to stay alive.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    sonar_param_cb_handle_;

  // Sonar fan-out parameters (see p2os.cpp for defaults/docs).
  bool sonar_publish_array_;
  bool sonar_publish_ranges_;
  bool sonar_publish_cloud_;
  std::string sonar_frame_prefix_;
  std::string sonar_parent_frame_;
  double sonar_fov_rad_;
  double sonar_min_range_m_;
  double sonar_max_range_m_;
  //! Z offset (m) applied to every per-sensor frame. The Pioneer
  //! sonar arrays physically sit ~0.25 m above base_link on the top
  //! plate; the robot_params sonar_pose table encodes only XY + yaw.
  //! Default 0.0 so behavior is unchanged for stacks that don't set
  //! it; grunt's launch overrides to the URDF top-plate height.
  double sonar_z_offset_m_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
  rclcpp::Subscription<p2os_msgs::msg::MotorState>::SharedPtr cmdmstate_sub_;
  rclcpp::Subscription<p2os_msgs::msg::GripperState>::SharedPtr gripper_sub_;
  rclcpp::Subscription<p2os_msgs::msg::PTZState>::SharedPtr ptz_cmd_sub_;
    
  P2OSPtz ptz_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  rclcpp::Time veltime;

  //! cmd_vel silence watchdog: if enabled and no cmd_vel arrives within
  //! cmd_vel_timeout_s_ seconds, the current non-zero velocity is zeroed
  //! to the wheels. 0.0 disables the watchdog entirely.
  double cmd_vel_timeout_s_;
  rclcpp::Time last_cmdvel_time_;
  bool cmdvel_watchdog_triggered_;

  SIP * sippacket;
  std::string psos_serial_port;
  std::string psos_tcp_host;
  std::string odom_frame_id;
  std::string base_link_frame_id;
  bool publish_tf_;
  int psos_fd;
  bool psos_use_tcp;
  int psos_tcp_port;
  bool vel_dirty, motor_dirty;
  bool gripper_dirty_ = false;
  int param_idx;
  // PID settings
  int rot_kp, rot_kv, rot_ki, trans_kp, trans_kv, trans_ki;

  //! Stall I hit a wall?
  int bumpstall;   // should we change the bumper-stall behavior?
  //! Use Joystick?
  int joystick;
  //! Control wheel velocities individually?
  int direct_wheel_vel_control;
  int radio_modemp;

  //! Maximum motor speed in Meters per second.
  int motor_max_speed;
  //! Maximum turn speed in radians per second.
  int motor_max_turnspeed;
  //! Maximum translational acceleration in Meters per second per second.
  int16_t motor_max_trans_accel;
  //! Minimum translational acceleration in Meters per second per second.
  int16_t motor_max_trans_decel;
  //! Maximum rotational acceleration in radians per second per second.
  int16_t motor_max_rot_accel;
  //! Minimum rotational acceleration in Meters per second per second.
  int16_t motor_max_rot_decel;
  //! Requested baud rate (0 = use robot params default)
  int requested_baud_rate;
  //! Pulse time
  double pulse;
  double desired_freq;
  //! Last time the node received or sent a pulse.
  double lastPulseTime;
  //! Use the sonar array?
  bool use_sonar_;

  //P2OSPtz ptz_;

public:
  //! Command Velocity subscriber
  geometry_msgs::msg::Twist cmdvel_;
  //! Motor state publisher
  p2os_msgs::msg::MotorState cmdmotor_state_;
  //! Gripper state publisher
  p2os_msgs::msg::GripperState gripper_state_;
  //! sensor data container
  ros_p2os_data_t p2os_data;
};

#endif  // P2OS_DRIVER__P2OS_HPP_
