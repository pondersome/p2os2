/*
 *  P2OS for ROS
 *  Copyright (C) 2009  David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *     Richard Vaughan, & Andrew Howard
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
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <p2os_driver/p2os.hpp>
#include <p2os_msgs/msg/motor_state.hpp>
//#include <tf/transform_datatypes.h>

#include <iostream>

// Main control loop, two nested whiles:
//
//   outer: Setup() / connection lifecycle. On Setup failure, sleep
//          briefly and retry — handles "USB symlink hasn't reappeared
//          yet" after a hot-plug. On serial error inside, CloseSerial()
//          and loop back to retry Setup().
//   inner: the existing per-tick work — check_and_set_vel,
//          check_and_set_motor_state, gripper, pulse, SendReceive,
//          spin_some. Unchanged in shape.
//
// Why this matters: before 2026-05-29, the driver entered Setup() once
// in main(), and a serial read EOF on a dropped USB device would (a) be
// invisible — see packet.cpp's old "if (cnt == 0) continue" hot loop —
// or (b) at best fall through SendReceive returning -1 and exit the
// process. With this reconnect loop the driver recovers in-process,
// preserving odometry continuity (sippacket persists across reconnects)
// and avoiding the systemd-restart latency.

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<P2OSNode>("p2os");

  bool first_connect = true;
  rclcpp::Time lastTime;

  while (rclcpp::ok()) {
    int setup_result = node->Setup();
    if (setup_result != 0) {
      // Setup failed. Common case during USB-drop recovery: udev
      // hasn't re-created /dev/grunt_p3at yet (re-enumeration takes
      // ~100-500 ms). Sleep interruptibly and retry. Bad-config cases
      // (wrong port, etc.) will also retry forever, but the operator
      // sees these as repeated WARN logs and can intervene; we don't
      // exit because that would also exit on a transient hot-plug at
      // boot.
      RCLCPP_WARN(rclcpp::get_logger("P2OsDriver"),
        "Setup() failed (rc=%d); retrying in 500 ms (USB symlink may "
        "not be settled yet, or chassis may be unreachable)",
        setup_result);
      for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
      continue;
    }

    if (first_connect) {
      // Only zero the encoders on the very first connect. On reconnect
      // (chassis power was maintained through the USB drop), the ARCOS
      // encoders are still accumulating from where they were; calling
      // ResetRawPositions would inject a SETO and zero them, producing
      // a phantom backward jump in odometry.
      node->ResetRawPositions();
      first_connect = false;
      lastTime = node->now();
    } else {
      RCLCPP_INFO(rclcpp::get_logger("P2OsDriver"),
        "Reconnected — preserving odometry offsets across the disconnect "
        "(SIP encoder diff will absorb any small jump if chassis didn't "
        "power-cycle)");
    }

    // Inner per-tick work loop. Exits on either:
    //   (a) rclcpp::ok() going false (Ctrl-C, SIGTERM from systemd)
    //   (b) SendReceive returning -1 (serial error → reconnect)
    while (rclcpp::ok()) {
      node->check_and_set_vel();
      node->check_and_set_motor_state();
      node->check_and_set_gripper_state();

      if (!rclcpp::ok()) { break; }

      if (node->get_pulse() > 0) {
        rclcpp::Time currentTime = node->now();
        rclcpp::Duration pulseInterval = currentTime - lastTime;
        if (pulseInterval.seconds() > node->get_pulse()) {
          RCLCPP_DEBUG(rclcpp::get_logger("P2OsDriver"), "sending pulse");
          node->SendPulse();
          lastTime = currentTime;
        }
      }

      // Hack fix to get around the fact that if no commands are sent to the
      // robot via SendReceive, the driver will never read SIP packets and so
      // never send data back to clients. We need a better way of doing regular
      // checks of the serial port - peek in sendreceive, maybe? Because if there
      // is no data waiting this will sit around waiting until one comes
      int sr = node->SendReceive(NULL, true);
      if (sr < 0) {
        RCLCPP_WARN(rclcpp::get_logger("P2OsDriver"),
          "Serial I/O error — closing dead fd and re-running Setup()");
        // Close the stale fd. Setup() will open a fresh one from the
        // (possibly newly re-pointed) udev symlink. Don't call
        // Shutdown() — that tries to send STOP/CLOSE bytes to a fd that
        // is already dead, and we want to KEEP sippacket alive for the
        // reconnect to inherit odometry state.
        node->CloseSerial();
        break;  // exit inner loop; outer loop re-enters Setup
      }
      if (!rclcpp::ok()) { break; }
      //node->updateDiagnostics();
      rclcpp::spin_some(node);
    }
    // If we got here because rclcpp::ok() went false, the outer while
    // will also see it and exit. If we got here because of a serial
    // error, the outer loop re-runs Setup().
  }

  node->Shutdown();

  RCLCPP_INFO(rclcpp::get_logger("P2OsDriver"), "Quitting... ");
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
