/*
 * teleop_base
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*       modifications to teleop_base to work with p2os
 *       Copyright (C) 2010  Hunter Allen [allen286@purdue.edu], David Feil-Seifer [dfseifer@usc.edu], Edward T. Kaszubski [kaszubsk@usc.edu]
 *
 *       This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *       the Free Software Foundation; either version 2 of the License, or
 *       (at your option) any later version.
 *       
 *       This program is distributed in the hope that it will be useful,
 *       but WITHOUT ANY WARRANTY; without even the implied warranty of
 *       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *       GNU General Public License for more details.
 *       
 *       You should have received a copy of the GNU General Public License
 *       along with this program; if not, write to the Free Software
 *       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *       MA 02110-1301, USA.
 */

#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TeleopBase : public rclcpp::Node
{
public:
    geometry_msgs::msg::Twist cmd, passthrough_cmd;
    double req_vx, req_vy, req_vw;
    double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
    int axis_vx, axis_vy, axis_vw;
    int deadman_button, run_button;
    bool deadman_no_publish_;
    bool deadman_;
    bool running_;
    rclcpp::Time last_received_joy_message_time_;
    rclcpp::Duration joy_msg_timeout_{-1,0};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr passthrough_sub_;

    TeleopBase(bool deadman_no_publish = false)
    : Node("p2os_teleop"), deadman_no_publish_(deadman_no_publish), running_(false)
    {
        this->declare_parameter<double>("max_vx", 0.6);
        this->declare_parameter<double>("max_vy", 0.6);
        this->declare_parameter<double>("max_vw", 0.8);
        this->declare_parameter<double>("max_vx_run", 0.6);
        this->declare_parameter<double>("max_vy_run", 0.6);
        this->declare_parameter<double>("max_vw_run", 0.8);
        this->declare_parameter<int>("axis_vx", 3);
        this->declare_parameter<int>("axis_vw", 0);
        this->declare_parameter<int>("axis_vy", 2);
        this->declare_parameter<int>("deadman_button", 0);
        this->declare_parameter<int>("run_button", 0);
        this->declare_parameter<double>("joy_msg_timeout", -1.0);

        max_vx = this->get_parameter("max_vx").as_double();
        max_vy = this->get_parameter("max_vy").as_double();
        max_vw = this->get_parameter("max_vw").as_double();
        max_vx_run = this->get_parameter("max_vx_run").as_double();
        max_vy_run = this->get_parameter("max_vy_run").as_double();
        max_vw_run = this->get_parameter("max_vw_run").as_double();
        axis_vx = this->get_parameter("axis_vx").as_int();
        axis_vw = this->get_parameter("axis_vw").as_int();
        axis_vy = this->get_parameter("axis_vy").as_int();
        deadman_button = this->get_parameter("deadman_button").as_int();
        run_button = this->get_parameter("run_button").as_int();
        double joy_msg_timeout = this->get_parameter("joy_msg_timeout").as_double();

        if (joy_msg_timeout <= 0)
        {
            joy_msg_timeout_ = rclcpp::Duration::max();
            RCLCPP_DEBUG(this->get_logger(), "joy_msg_timeout <= 0 -> no timeout");
        }
        else
        {
            joy_msg_timeout_ = rclcpp::Duration::from_seconds(joy_msg_timeout);
            RCLCPP_DEBUG(this->get_logger(), "joy_msg_timeout: %.3f", joy_msg_timeout_.seconds());
        }

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        passthrough_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "des_vel", 10, std::bind(&TeleopBase::passthrough_cb, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TeleopBase::joy_cb, this, std::placeholders::_1));
    }

    void passthrough_cb(const geometry_msgs::msg::Twist::SharedPtr pass_msg)
    {
        passthrough_cmd = *pass_msg;
    }

    void joy_cb(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        deadman_ = (static_cast<size_t>(deadman_button) < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button];

        if (!deadman_)
            return;

        last_received_joy_message_time_ = this->now();

        running_ = (static_cast<size_t>(run_button) < joy_msg->buttons.size()) && joy_msg->buttons[run_button];
        double vx = running_ ? max_vx_run : max_vx;
        double vy = running_ ? max_vy_run : max_vy;
        double vw = running_ ? max_vw_run : max_vw;

        if ((axis_vx >= 0) && (static_cast<size_t>(axis_vx) < joy_msg->axes.size()))
            req_vx = joy_msg->axes[axis_vx] * vx;
        if ((axis_vy >= 0) && (static_cast<size_t>(axis_vy) < joy_msg->axes.size()))
            req_vy = joy_msg->axes[axis_vy] * vy;
        if ((axis_vw >= 0) && (static_cast<size_t>(axis_vw) < joy_msg->axes.size()))
            req_vw = joy_msg->axes[axis_vw] * vw;
    }

    void send_cmd_vel()
    {
        if (deadman_ && (last_received_joy_message_time_ + joy_msg_timeout_ > this->now()) && running_) {
            cmd.linear.x = req_vx;
            cmd.linear.y = req_vy;
            cmd.angular.z = req_vw;
            vel_pub_->publish(cmd);
        } else {
            cmd = passthrough_cmd;
            if (!deadman_no_publish_)
                vel_pub_->publish(cmd);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopBase>(std::find(argv, argv + argc, std::string("--deadman_no_publish")) != argv + argc);
    rclcpp::Rate rate(10); // 10 Hz

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->send_cmd_vel();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
