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

///\author Kevin Watts
///\brief Converts joystick commands on /joy to commands to PR2 base, spine, head

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "topic_tools/MuxSelect.h"
#include "std_msgs/String.h"

#include "std_msgs/Float64.h"

#define TORSO_TOPIC "torso_controller/command"
#define HEAD_TOPIC "head_traj_controller/command"
const int PUBLISH_FREQ = 20;

using namespace std;

class TeleopPR2
{
   public:
  geometry_msgs::Twist cmd;
  double min_torso, max_torso;
  double req_torso_vel, torso_step;
  //joy::Joy joy;
  double req_vx, req_vy, req_vw, req_torso, req_pan, req_tilt;
  double req_tilt_vel, req_pan_vel;
  double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
  double max_pan, max_tilt, min_tilt, pan_step, tilt_step;
  int axis_vx, axis_vy, axis_vw, axis_pan, axis_tilt;
  int deadman_button, run_button, torso_dn_button, torso_up_button, head_button;
  bool deadman_no_publish_, torso_publish_, head_publish_;

  bool deadman_, cmd_head;
  bool use_mux_, last_deadman_;
  std::string last_selected_topic_;

  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher vel_pub_;
  ros::Publisher head_pub_;
  ros::Publisher torso_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber torso_state_sub_;
  ros::ServiceClient mux_client_;

  TeleopPR2(bool deadman_no_publish = false) :
    max_vx(0.6), max_vy(0.6), max_vw(0.8),
    max_vx_run(0.6), max_vy_run(0.6), max_vw_run(0.8),
    max_pan(2.7), max_tilt(1.4), min_tilt(-0.4),
    pan_step(0.02), tilt_step(0.015),
    deadman_no_publish_(deadman_no_publish), 
    torso_publish_(false), head_publish_(false),
    deadman_(false), cmd_head(false), 
    use_mux_(false), last_deadman_(false),
    n_private_("~")
  { }

  void init()
  {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
        req_pan = req_tilt = 0;
        req_torso = 0.0;
        req_torso_vel = 0.0;

        //parameters for interaction with a mux on cmd_vel topics
        n_private_.param("use_mux", use_mux_, false);

        n_private_.param("max_vx", max_vx, max_vx);
        n_private_.param("max_vy", max_vy, max_vy);
        n_private_.param("max_vw", max_vw, max_vw);

        // Set max speed while running
        n_private_.param("max_vx_run", max_vx_run, max_vx_run);
        n_private_.param("max_vy_run", max_vy_run, max_vy_run);
        n_private_.param("max_vw_run", max_vw_run, max_vw_run);

        // Head pan/tilt parameters
        n_private_.param("max_pan", max_pan, max_pan);
        n_private_.param("max_tilt", max_tilt, max_tilt);
        n_private_.param("min_tilt", min_tilt, min_tilt);

        n_private_.param("tilt_step", tilt_step, tilt_step);
        n_private_.param("pan_step", pan_step, pan_step);

        n_private_.param("axis_pan", axis_pan, 0);
        n_private_.param("axis_tilt", axis_tilt, 2);

        n_private_.param("axis_vx", axis_vx, 3);
        n_private_.param("axis_vw", axis_vw, 0);
        n_private_.param("axis_vy", axis_vy, 2);

        n_private_.param("torso_step", torso_step, 0.01);
        n_private_.param("min_torso", min_torso, 0.0);
        n_private_.param("max_torso", max_torso, 0.3);

        n_private_.param("deadman_button", deadman_button, 0);
        n_private_.param("run_button", run_button, 0);
        n_private_.param("torso_dn_button", torso_dn_button, 0);
        n_private_.param("torso_up_button", torso_up_button, 0);
        n_private_.param("head_button", head_button, 0);

	double joy_msg_timeout;
        n_private_.param("joy_msg_timeout", joy_msg_timeout, 0.5); //default to 0.5 seconds timeout
	if (joy_msg_timeout <= 0)
	  {
	    joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
	    ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
	  }
	else
	  {
	    joy_msg_timeout_.fromSec(joy_msg_timeout);
	    ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
	  }

        ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
        ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
        ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);

        ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
        ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
        ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);

        ROS_DEBUG("tilt step: %.3f rad\n", tilt_step);
        ROS_DEBUG("pan step: %.3f rad\n", pan_step);

        ROS_DEBUG("axis_vx: %d\n", axis_vx);
        ROS_DEBUG("axis_vy: %d\n", axis_vy);
        ROS_DEBUG("axis_vw: %d\n", axis_vw);
        ROS_DEBUG("axis_pan: %d\n", axis_pan);
        ROS_DEBUG("axis_tilt: %d\n", axis_tilt);

        ROS_DEBUG("deadman_button: %d\n", deadman_button);
        ROS_DEBUG("run_button: %d\n", run_button);
        ROS_DEBUG("torso_dn_button: %d\n", torso_dn_button);
        ROS_DEBUG("torso_up_button: %d\n", torso_up_button);
        ROS_DEBUG("head_button: %d\n", head_button);
        ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);

        if (torso_dn_button != 0 && torso_up_button != 0)
        {
          torso_publish_ = true;
          torso_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(TORSO_TOPIC, 1);
        }

        if (head_button != 0)
        {
          head_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(HEAD_TOPIC, 1);
          head_publish_ = true;
        }

        vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        joy_sub_ = n_.subscribe("joy", 10, &TeleopPR2::joy_cb, this);
        torso_state_sub_ = n_.subscribe("torso_controller/state", 1, &TeleopPR2::torsoCB, this);

        //if we're going to use the mux, then we'll subscribe to state changes on the mux
        if(use_mux_){
          ros::NodeHandle mux_nh("mux");
          mux_client_ = mux_nh.serviceClient<topic_tools::MuxSelect>("select");
        }
      }

  ~TeleopPR2() { }

  /** Callback for joy topic **/
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    //Record this message reciept
    last_recieved_joy_message_time_ = ros::Time::now();

    deadman_ = (((unsigned int)deadman_button < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button]);

    if (!deadman_)
      return;

    cmd_head = (((unsigned int)head_button < joy_msg->buttons.size()) && joy_msg->buttons[head_button] && head_publish_);

    // Base
    bool running = (((unsigned int)run_button < joy_msg->buttons.size()) && joy_msg->buttons[run_button]);
    double vx = running ? max_vx_run : max_vx;
    double vy = running ? max_vy_run : max_vy;
    double vw = running ? max_vw_run : max_vw;

    if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()) && !cmd_head)
      req_vx = joy_msg->axes[axis_vx] * vx;
    else
      req_vx = 0.0;
    if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()) && !cmd_head)
      req_vy = joy_msg->axes[axis_vy] * vy;
    else
      req_vy = 0.0;
    if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy_msg->axes.size()) && !cmd_head)
      req_vw = joy_msg->axes[axis_vw] * vw;
    else
      req_vw = 0.0;

    // Enforce max/mins for velocity
    // Joystick should be [-1, 1], but it might not be
    req_vx = max(min(req_vx, vx), -vx);
    req_vy = max(min(req_vy, vy), -vy);
    req_vw = max(min(req_vw, vw), -vw);

    // Head
    // Update commanded position by how joysticks moving
    // Don't add commanded position if deadman off
    if (deadman_ && cmd_head)
    {
      if (axis_pan >= 0 && axis_pan < (int)joy_msg->axes.size())
      {
        req_pan_vel = joy_msg->axes[axis_pan] * pan_step;
      }

      if (axis_tilt >= 0 && axis_tilt < (int)joy_msg->axes.size())
      {
        req_tilt_vel = joy_msg->axes[axis_tilt] * tilt_step;
      }
    }

    // Torso
    bool down = (((unsigned int)torso_dn_button < joy_msg->buttons.size()) && joy_msg->buttons[torso_dn_button]);
    bool up = (((unsigned int)torso_up_button < joy_msg->buttons.size()) && joy_msg->buttons[torso_up_button]);

    // Bring torso up/down
    if (down && !up)
      req_torso_vel = -torso_step;
    else if (up && !down)
      req_torso_vel = torso_step;
    else
      req_torso_vel = 0;
  }


  void send_cmd_vel()
  {
    if(deadman_  &&
       last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
    {
      //check if we need to switch the mux to our topic for teleop
      if(use_mux_ && !last_deadman_){
        topic_tools::MuxSelect select_srv;
        select_srv.request.topic = vel_pub_.getTopic();
        if(mux_client_.call(select_srv)){
          last_selected_topic_ = select_srv.response.prev_topic;
          ROS_DEBUG("Setting mux to %s for teleop", select_srv.request.topic.c_str());
        }
        else{
          ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
        }
      }

      // Base
      cmd.linear.x = req_vx;
      cmd.linear.y = req_vy;
      cmd.angular.z = req_vw;
      vel_pub_.publish(cmd);

      // Torso
      if (torso_publish_)
      {
        double dt = 1.0/double(PUBLISH_FREQ);
        double horizon = 5.0 * dt;

        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
        traj.joint_names.push_back("torso_lift_joint");
        traj.points.resize(1);
        traj.points[0].positions.push_back(req_torso + req_torso_vel * horizon);
        traj.points[0].velocities.push_back(req_torso_vel);
        traj.points[0].time_from_start = ros::Duration(horizon);
        torso_pub_.publish(traj);

        // Updates the current positions
        req_torso += req_torso_vel * dt;
        req_torso = max(min(req_torso, max_torso), min_torso);
      }

      // Head
      if (cmd_head && head_publish_)
      {
        double dt = 1.0/double(PUBLISH_FREQ);
        double horizon = 3.0 * dt;

        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
        traj.joint_names.push_back("head_pan_joint");
        traj.joint_names.push_back("head_tilt_joint");
        traj.points.resize(1);
        traj.points[0].positions.push_back(req_pan + req_pan_vel * horizon);
        traj.points[0].velocities.push_back(req_pan_vel);
        traj.points[0].positions.push_back(req_tilt + req_tilt_vel * horizon);
        traj.points[0].velocities.push_back(req_tilt_vel);
        traj.points[0].time_from_start = ros::Duration(horizon);
        head_pub_.publish(traj);

        // Updates the current positions
        req_pan += req_pan_vel * dt;
        req_pan = max(min(req_pan, max_pan), -max_pan);
        req_tilt += req_tilt_vel * dt;
        req_tilt = max(min(req_tilt, max_tilt), min_tilt);
      }

      if (req_torso != 0)
        fprintf(stdout,"teleop_base:: %f, %f, %f. Head:: %f, %f. Torso cmd: %f.\n",
                cmd.linear.x, cmd.linear.y, cmd.angular.z, req_pan, req_tilt, req_torso_vel);
      else
        fprintf(stdout,"teleop_base:: %f, %f, %f. Head:: %f, %f\n",
                cmd.linear.x ,cmd.linear.y, cmd.angular.z, req_pan, req_tilt);
    }
    else
    {
      //make sure to set the mux back to whatever topic it was on when we grabbed it if the deadman has just toggled
      if(use_mux_ && last_deadman_){
        topic_tools::MuxSelect select_srv;
        select_srv.request.topic = last_selected_topic_;
        if(mux_client_.call(select_srv)){
          ROS_DEBUG("Setting mux back to %s", last_selected_topic_.c_str());
        }
        else{
          ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
        }
      }

      // Publish zero commands iff deadman_no_publish is false
      cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
      if (!deadman_no_publish_)
      {
        // Base
        vel_pub_.publish(cmd);
      }
    }

    //make sure we store the state of our last deadman
    last_deadman_ = deadman_;
  }

  void torsoCB(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr &msg)
  {
    double xd = req_torso;
    const double A = 0.003;
    if (fabs(msg->actual.positions[0] - xd) > A*1.001)
    {
      req_torso = min(max(msg->actual.positions[0] - A, xd), msg->actual.positions[0] + A);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_pr2");
  const char* opt_no_publish    = "--deadman_no_publish";

  bool no_publish = false;
  for(int i=1;i<argc;i++)
  {
    if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
      no_publish = true;
  }

  TeleopPR2 teleop_pr2(no_publish);
  teleop_pr2.init();

  ros::Rate pub_rate(PUBLISH_FREQ);

  while (teleop_pr2.n_.ok())
  {
    ros::spinOnce();
    teleop_pr2.send_cmd_vel();
    pub_rate.sleep();
  }

  exit(0);
  return 0;
}

