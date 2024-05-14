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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<P2OSNode>("p2os");
  //P2OSNode * p = new P2OSNode(n);//ros1 way
    

  if (node->Setup()) {
    RCLCPP_ERROR(rclcpp::get_logger("P2OsDriver"), "p2os setup failed...");
    return -1;
  }

  node->ResetRawPositions();

  rclcpp::Time lastTime;

  while (rclcpp::ok()) {
    node->check_and_set_vel();
    node->check_and_set_motor_state();
    node->check_and_set_gripper_state();

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
    node->SendReceive(NULL, true);
    //node->updateDiagnostics();
    rclcpp::spin_some(node);
  }

  if (!node->Shutdown()) {
    RCLCPP_WARN(rclcpp::get_logger("P2OsDriver"), "p2os shutdown failed... your robot might be heading for the wall?");
  }
  //delete p; //std::shared_ptr knows how to cleanup itself

  RCLCPP_INFO(rclcpp::get_logger("P2OsDriver"), "Quitting... ");
  return 0;
}
