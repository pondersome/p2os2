/*
 * Software License Agreement (BSD License)
 *
 *  Hunter Allen <hunter.allen@vanderbilt.edu>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StatePublisher3AT : public rclcpp::Node
{
public:
  StatePublisher3AT() : Node("state_publisher")
  {
    joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(33ms, std::bind(&StatePublisher3AT::update, this));
  }

private:
  void update()
  {
    auto joint_state = sensor_msgs::msg::JointState{};
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name.resize(4); // Assume 4 joints as before
    joint_state.position.resize(4);

    joint_state.name[0] = "p3at_back_right_wheel_joint";
    joint_state.position[0] = 0;

    joint_state.name[1] = "p3at_back_left_wheel_joint";
    joint_state.position[1] = 0;

    joint_state.name[2] = "p3at_front_left_wheel_joint";
    joint_state.position[2] = 0;

    joint_state.name[3] = "p3at_front_right_wheel_joint";
    joint_state.position[3] = 0;

    joint_state_publisher->publish(joint_state);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisher3AT>());
  rclcpp::shutdown();
  return 0;
}
