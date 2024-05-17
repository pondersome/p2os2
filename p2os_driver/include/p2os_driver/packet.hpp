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
#ifndef P2OS_DRIVER__PACKET_HPP_
#define P2OS_DRIVER__PACKET_HPP_
#include "rclcpp/rclcpp.hpp"

#include <cstring>

namespace
{
constexpr size_t packet_len = 256;
}

class P2OSPacket
{
private:
     rclcpp::Logger logger_;
     rclcpp::Clock clock_;
public:
  //P2OSPacket(rclcpp::Node::SharedPtr node) : node_(node) {} //i think the magical ros tool thought this was supposed to be a node
  P2OSPacket() : logger_(rclcpp::get_logger("p2os_driver")), clock_(RCL_ROS_TIME) {
    // Debug: Print the current time to check clock initialization
    // Get the current time
    rclcpp::Time current_time = clock_.now();

    // Convert the time to a string
    //std::string time_str = std::to_string(current_time.seconds());
    // Log the time string
    //RCLCPP_DEBUG(logger_, "Time at packet allocation: %s", time_str.c_str());
  }

  //rclcpp::Node::SharedPtr node_; //seems like this is only used to get a logger connected to the node that instantiated the packet?
  unsigned char packet[packet_len];
  unsigned char size;
  rclcpp::Time timestamp;

  int CalcChkSum();

  void Print();
  void PrintHex();
  int Build(unsigned char * data, unsigned char datasize);
  int Send(int fd);
  int Receive(int fd);
  bool Check();

  bool operator!=(P2OSPacket p)
  {
    if (size != p.size) {return true;}

    if (memcmp(packet, p.packet, size) != 0) {return true;}

    return false;
  }
};

#endif  // P2OS_DRIVER__PACKET_HPP_
