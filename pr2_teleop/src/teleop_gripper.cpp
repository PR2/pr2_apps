/*
 * teleop_pr2
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

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <joy/Joy.h>

class TeleopGripper
{
public:
  TeleopGripper()
  {
    ros::NodeHandle private_nh_("~");

    private_nh_.param("open_position",   open_cmd_.position,     0.08);
    private_nh_.param("open_max_effort", open_cmd_.max_effort,  -1.0);

    private_nh_.param("close_position",   close_cmd_.position,  -100.00);
    private_nh_.param("close_max_effort", close_cmd_.max_effort,   -1.0);

    private_nh_.param("open_button",   open_button_,   1);
    private_nh_.param("close_button",  close_button_,  2);

    pub_ = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>("command", 1, false);
    sub_ = nh_.subscribe("joy", 1, &TeleopGripper::joyCallback, this);

    ROS_DEBUG("teleop_gripper started");
  }

  void joyCallback(const joy::JoyConstPtr& joy)
  {
    ROS_DEBUG("Got a joy msg");
    if ((int) joy->buttons.size() <= open_button_ ||
        (int) joy->buttons.size() <= close_button_ ||
        open_button_ < 0 ||
        close_button_ < 0)
    {
      ROS_ERROR("Array lookup error: Joystick message has %u elems. Open Index [%u]. Close Index [%u]", (unsigned int)joy->buttons.size(), open_button_, close_button_);
      return;
    }

    ROS_DEBUG("open: Buttons[%u] = %u", open_button_, joy->buttons[open_button_]);

    if (joy->buttons[open_button_] == 1)
    {
      pub_.publish(open_cmd_);
      ROS_DEBUG("Opening");
    }
    else if (joy->buttons[close_button_] == 1)
    {
      pub_.publish(close_cmd_);
      ROS_DEBUG("Closing");
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  int close_button_;
  int open_button_;


  pr2_controllers_msgs::Pr2GripperCommand open_cmd_;
  pr2_controllers_msgs::Pr2GripperCommand close_cmd_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_gripper");

  TeleopGripper teleop_gripper;

  ros::spin();

}
