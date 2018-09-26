/*
 * pr2_teleop_booth_commander
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

// Author: E. Gil Jones

#include <string>
#include <boost/bind.hpp>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <pr2_mechanism_msgs/SwitchController.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <polled_camera/GetPolledImage.h>

#include "pr2_teleop_general/pr2_teleop_general_commander.h"

#include "urdf_model/pose.h"

static const std::string LEFT_HAND_LINK_TO_TRACK = "l_gripper_palm_link";
static const std::string RIGHT_HAND_LINK_TO_TRACK = "r_gripper_palm_link";

static const double MAX_HEAD_TRACK_SPEED = 2.0;

static const double GRIPPER_CLOSE_POSITION = 0.000;
static const double GRIPPER_CLOSE_MAX_EFFORT = 10000.0;

static const double GRIPPER_OPEN_POSITION = 0.086;
static const double GRIPPER_OPEN_MAX_EFFORT = 10000.0;

static const std::string RIGHT_ARM_MANNEQUIN_CONTROLLER = "r_arm_controller_loose";
static const std::string LEFT_ARM_MANNEQUIN_CONTROLLER = "l_arm_controller_loose";

static const std::string HEAD_MANNEQUIN_CONTROLLER = "head_traj_controller_loose";
static const std::string HEAD_POSITION_CONTROLLER = "head_traj_controller";

static const unsigned int WALK_BUFFER = 10;

GeneralCommander::GeneralCommander(bool control_body,
                                   bool control_head,
                                   bool control_rarm,
                                   bool control_larm,
                                   bool control_prosilica,
                                   std::string arm_controller_name) 
  : n_(),
    control_body_(control_body),
    control_head_(control_head),
    control_rarm_(control_rarm),
    control_larm_(control_larm),
    control_prosilica_(control_prosilica)                                                        
{

  r_arm_controller_name_ = "r_"+arm_controller_name;
  l_arm_controller_name_ = "l_"+arm_controller_name;

  head_control_mode_ = HEAD_JOYSTICK;      

  std::string urdf_xml,full_urdf_xml;
  n_.param("urdf_xml", urdf_xml, std::string("robot_description"));
  if(!n_.getParam(urdf_xml,full_urdf_xml))
  {
    ROS_ERROR("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    robot_model_initialized_ = false;
  }
  else
  {
    robot_model_.initString(full_urdf_xml);
    robot_model_initialized_ = true;
  }

  //universal
  switch_controllers_service_ = n_.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
  joint_state_sub_ = n_.subscribe("joint_states", 1, &GeneralCommander::jointStateCallback, this);
  power_board_sub_ = n_.subscribe<pr2_msgs::PowerBoardState>("power_board/state", 1, &GeneralCommander::powerBoardCallback, this);

  if(control_head_) {
    tilt_laser_service_ = n_.serviceClient<pr2_msgs::SetPeriodicCmd>("laser_tilt_controller/set_periodic_cmd");
    head_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("head_traj_controller/command", 1);  
    head_track_hand_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>("/head_traj_controller/point_head_action", true);
    while(!head_track_hand_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
    //just in case there's an existing goal
    head_track_hand_client_->cancelAllGoals();
  } else {
    head_track_hand_client_ = NULL;
  }
  if(control_body_) {
    torso_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("torso_controller/command", 1);
    base_pub_ = n_.advertise<geometry_msgs::Twist>("base_controller/command", 1);
  }
  if(control_rarm_) {
    right_gripper_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("r_gripper_controller/gripper_action", true);
    right_arm_trajectory_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(r_arm_controller_name_+"/joint_trajectory_action", true);
    right_arm_traj_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(r_arm_controller_name_+"/command", 1);
    while(!right_gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the right gripper action server to come up");
    }  
    while(!right_arm_trajectory_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO_STREAM("Waiting for the right arm trajectory action server to come up" << r_arm_controller_name_+"/joint_trajectory_action");
    }
  } else {
    right_gripper_client_ = NULL;
    right_arm_trajectory_client_ = NULL;
  }
  if(control_larm_) {
    left_gripper_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("l_gripper_controller/gripper_action", true);
    left_arm_trajectory_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(l_arm_controller_name_+"/joint_trajectory_action", true);
    left_arm_traj_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(l_arm_controller_name_+"/command", 1);
    while(!left_gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the right gripper action server to come up");
    }
    while(!left_arm_trajectory_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO_STREAM("Waiting for the left arm trajectory action server to come up");
    }
  } else {
    left_gripper_client_ = NULL;
    left_arm_trajectory_client_ = NULL;
  }
  if(control_larm_ || control_rarm_) {
    tuck_arms_client_ = new actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction>("tuck_arms", true);
  } else {
    tuck_arms_client_ = NULL;
  }

  if(control_rarm_) {
    ROS_INFO("Waiting for right arm kinematics servers");
    ros::service::waitForService("pr2_right_arm_kinematics/get_fk");
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
    right_arm_kinematics_forward_client_ = n_.serviceClient<moveit_msgs::GetPositionFK>("pr2_right_arm_kinematics/get_fk", true);
    right_arm_kinematics_inverse_client_ = n_.serviceClient<moveit_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik", true);
  }

  if(control_larm_) {
    ROS_INFO("Waiting for left arm kinematics servers");
    ros::service::waitForService("pr2_left_arm_kinematics/get_fk");
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik");
    left_arm_kinematics_forward_client_ = n_.serviceClient<moveit_msgs::GetPositionFK>("pr2_left_arm_kinematics/get_fk", true);
    left_arm_kinematics_inverse_client_ = n_.serviceClient<moveit_msgs::GetPositionIK>("pr2_left_arm_kinematics/get_ik", true);
  }
  

  if(control_prosilica_) {
    ros::service::waitForService("prosilica/request_image");
    prosilica_polling_client_ = n_.serviceClient<polled_camera::GetPolledImage>("prosilica/request_image", true);
  }

  right_walk_along_pose_.push_back(.049);
  right_walk_along_pose_.push_back(-.116);
  right_walk_along_pose_.push_back(.036);
  right_walk_along_pose_.push_back(-1.272);
  right_walk_along_pose_.push_back(-.084);
  right_walk_along_pose_.push_back(-.148);
  right_walk_along_pose_.push_back(-0.027);

  left_walk_along_pose_.push_back(0.0);
  left_walk_along_pose_.push_back(-.149);
  left_walk_along_pose_.push_back(.085);
  left_walk_along_pose_.push_back(-1.234);
  left_walk_along_pose_.push_back(0.030);
  left_walk_along_pose_.push_back(-.141);
  left_walk_along_pose_.push_back(3.114);

  head_nod_traj_.joint_names.push_back("head_pan_joint");
  head_nod_traj_.joint_names.push_back("head_tilt_joint");
  head_nod_traj_.points.resize(3);
  head_nod_traj_.points[0].positions.push_back(0.0);
  head_nod_traj_.points[0].positions.push_back(-0.2);
  head_nod_traj_.points[0].velocities.push_back(0.0);
  head_nod_traj_.points[0].velocities.push_back(0.0);
  head_nod_traj_.points[0].time_from_start = ros::Duration(0.5);

  head_nod_traj_.points[1].positions.push_back(0.0);
  head_nod_traj_.points[1].positions.push_back(0.2);
  head_nod_traj_.points[1].velocities.push_back(0.0);
  head_nod_traj_.points[1].velocities.push_back(0.0);
  head_nod_traj_.points[1].time_from_start = ros::Duration(1.0);

  head_nod_traj_.points[2].positions.push_back(0.0);
  head_nod_traj_.points[2].positions.push_back(0.0);
  head_nod_traj_.points[2].velocities.push_back(0.0);
  head_nod_traj_.points[2].velocities.push_back(0.0);
  head_nod_traj_.points[2].time_from_start = ros::Duration(1.5);

  head_shake_traj_.joint_names.push_back("head_pan_joint");
  head_shake_traj_.joint_names.push_back("head_tilt_joint");
  head_shake_traj_.points.resize(3);
  head_shake_traj_.points[0].positions.push_back(.62);
  head_shake_traj_.points[0].positions.push_back(0.0);
  head_shake_traj_.points[0].velocities.push_back(0.0);
  head_shake_traj_.points[0].velocities.push_back(0.0);
  head_shake_traj_.points[0].time_from_start = ros::Duration(0.5);

  head_shake_traj_.points[1].positions.push_back(-.62);
  head_shake_traj_.points[1].positions.push_back(0.0);
  head_shake_traj_.points[1].velocities.push_back(0.0);
  head_shake_traj_.points[1].velocities.push_back(0.0);
  head_shake_traj_.points[1].time_from_start = ros::Duration(1.0);

  head_shake_traj_.points[2].positions.push_back(0.0);
  head_shake_traj_.points[2].positions.push_back(0.0);
  head_shake_traj_.points[2].velocities.push_back(0.0);
  head_shake_traj_.points[2].velocities.push_back(0.0);
  head_shake_traj_.points[2].time_from_start = ros::Duration(1.5);

  //making sure that everything is in the right mode
  right_arm_control_mode_ = ARM_MANNEQUIN_MODE;
  left_arm_control_mode_ = ARM_MANNEQUIN_MODE;
  setArmMode(ARMS_BOTH, ARM_POSITION_CONTROL);
  head_control_mode_ = HEAD_MANNEQUIN;
  if(control_head_) {
    setHeadMode(HEAD_JOYSTICK);
    laser_control_mode_ = LASER_TILT_OFF;
  }
    
  //cribbed from motion planning laser settings
  laser_slow_period_ = 10;
  laser_slow_amplitude_ = .75;
  laser_slow_offset_ = .25;
  
  //cribbed from head cart tilt_laser_launch
  laser_fast_period_ = 2;
  laser_fast_amplitude_ = .78;
  laser_fast_offset_ = .3;

  last_right_wrist_goal_stamp_ = ros::Time::now();
  last_left_wrist_goal_stamp_ = ros::Time::now();

  last_torso_vel_ = 0.0;
  walk_along_ok_ = false;
}

GeneralCommander::~GeneralCommander() {
  if(head_track_hand_client_) {
    head_track_hand_client_->cancelAllGoals();
    delete head_track_hand_client_;
  } 
  if(right_gripper_client_) {
    delete right_gripper_client_;
  }
  if(left_gripper_client_) {
    delete left_gripper_client_;
  }
  if(right_arm_trajectory_client_) {
    delete right_arm_trajectory_client_;
  }
  if(left_arm_trajectory_client_) {
    delete left_arm_trajectory_client_;
  }
  if(tuck_arms_client_) {
    delete tuck_arms_client_;
  }
}

void GeneralCommander::jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState)
{
  for(unsigned int i = 0; i < jointState->name.size(); i++) {
    joint_state_position_map_[jointState->name[i]] = jointState->position[i];
    joint_state_velocity_map_[jointState->name[i]] = jointState->velocity[i];
    //if(jointState->name[i] == "r_wrist_roll_joint") {
    //  ROS_INFO_STREAM("Right wrist roll pos " <<  jointState->position[i] << " vel " <<  jointState->velocity[i]);
    //}
  }
}

bool GeneralCommander::getJointPosition(const std::string& name, double& pos) const {
  if(joint_state_position_map_.find(name) == joint_state_position_map_.end()) {
    return false;
  }
  pos = joint_state_position_map_.find(name)->second;
  return true;
}

bool GeneralCommander::getJointVelocity(const std::string& name, double& vel) const {
  if(joint_state_velocity_map_.find(name) == joint_state_velocity_map_.end()) {
    return false;
  }
  vel = joint_state_velocity_map_.find(name)->second;
  return true;
}

GeneralCommander::LaserControlMode GeneralCommander::getLaserMode() {
  return laser_control_mode_;
}

GeneralCommander::HeadControlMode GeneralCommander::getHeadMode() {
  return head_control_mode_;
}

GeneralCommander::ArmControlMode GeneralCommander::getArmMode(WhichArm arm) {
  if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
    return right_arm_control_mode_;
  } else {
    return left_arm_control_mode_;
  }
}

void GeneralCommander::sendProjectorStartStop(bool start) {
  
  if(!control_head_) return;

  if(start) {
    int ok = system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node narrow_stereo_trig_mode 3"); 
    ROS_DEBUG("Trying to send projector on");
    if(ok < 0) {
      ROS_WARN("Dynamic reconfigure for setting trigger mode ON failed");
    } 
  } else {
    int ok = system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node narrow_stereo_trig_mode 4");
    ROS_DEBUG("Trying to send trigger off");
    if(ok < 0) {
      ROS_WARN("Dynamic reconfigure for setting trigger mode OFF failed");
    }
  }
}

void GeneralCommander::setLaserMode(LaserControlMode mode) {
  if(!control_head_) return;

  if(laser_control_mode_ == mode) return;

  pr2_msgs::SetPeriodicCmd::Request req;
  pr2_msgs::SetPeriodicCmd::Response res;
  
  req.command.profile = "linear";
  if(mode == LASER_TILT_SLOW) {
    ROS_DEBUG("Sending slow");
    req.command.period = laser_slow_period_;
    req.command.amplitude = laser_slow_period_;
    req.command.offset = laser_slow_offset_;
  } else if(mode == LASER_TILT_FAST) {
    ROS_DEBUG("Sending fast");
    req.command.period = laser_fast_period_;
    req.command.amplitude = laser_fast_period_;
    req.command.offset = laser_fast_offset_;
  } else {
    ROS_DEBUG("Sending off");
    req.command.period = 1.0;
    req.command.amplitude = 0;
    req.command.offset = laser_slow_offset_;
  }
  
  if(tilt_laser_service_.call(req,res)) {
    //ROS_DEBUG("Resp start time is %g", res.start_time.toSeconds()); 
  } else {
    ROS_ERROR("Tilt laser service call failed.\n");
  }
  laser_control_mode_ = mode;
}

void GeneralCommander::setHeadMode(HeadControlMode mode) {
  if(!control_head_) return;
  if(mode == head_control_mode_) return;
  if(mode == HEAD_TRACK_LEFT_HAND) {
    ROS_DEBUG("Setting head to track left hand");
  } else if(mode == HEAD_TRACK_RIGHT_HAND) {
    ROS_DEBUG("Setting head to track right hand");
  }
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  if(mode == HEAD_MANNEQUIN) {
    start_controllers.push_back(HEAD_MANNEQUIN_CONTROLLER);
    stop_controllers.push_back(HEAD_POSITION_CONTROLLER);
  } else if(head_control_mode_ == HEAD_MANNEQUIN) {
    start_controllers.push_back(HEAD_POSITION_CONTROLLER);
    stop_controllers.push_back(HEAD_MANNEQUIN_CONTROLLER);
  }
  if(!start_controllers.empty() || !stop_controllers.empty()) {
    switchControllers(start_controllers, stop_controllers);
  }
  head_control_mode_ = mode;
}

void GeneralCommander::setArmMode(WhichArm arm, ArmControlMode mode) {
  if(!control_rarm_ && !control_larm_) return;
  if(!control_rarm_ && arm == ARMS_RIGHT) return;
  if(!control_larm_ && arm == ARMS_LEFT) return;

  if(arm == ARMS_LEFT) {
    if(mode == left_arm_control_mode_) return;
  } else if(arm == ARMS_RIGHT) {
    if(mode == right_arm_control_mode_) return;
  } else {
    if(mode == left_arm_control_mode_ && mode == right_arm_control_mode_) return;
  }

  std::string left_running_controller;
  std::string right_running_controller;

  if(left_arm_control_mode_ == ARM_MANNEQUIN_MODE) {
    left_running_controller = LEFT_ARM_MANNEQUIN_CONTROLLER;
  } else if(left_arm_control_mode_ == ARM_POSITION_CONTROL) {
    left_running_controller = l_arm_controller_name_;
  }

  if(right_arm_control_mode_ == ARM_MANNEQUIN_MODE) {
    right_running_controller = RIGHT_ARM_MANNEQUIN_CONTROLLER;
  } else if(right_arm_control_mode_ == ARM_POSITION_CONTROL) {
    right_running_controller = r_arm_controller_name_;
  }

  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;

  if(mode == ARM_NO_CONTROLLER) {
    if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
      stop_controllers.push_back(left_running_controller);
    }
    if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
      stop_controllers.push_back(right_running_controller);
    }
  } else if(mode == ARM_MANNEQUIN_MODE) {
    if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
      if(!left_running_controller.empty()) {
        stop_controllers.push_back(left_running_controller);
      }
      start_controllers.push_back(LEFT_ARM_MANNEQUIN_CONTROLLER);
    }
    if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
      if(!right_running_controller.empty()) {
        stop_controllers.push_back(right_running_controller);
      }
      start_controllers.push_back(RIGHT_ARM_MANNEQUIN_CONTROLLER);
    }
  } else if(mode == ARM_POSITION_CONTROL) {
    if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
      if(!left_running_controller.empty()) {
        stop_controllers.push_back(left_running_controller);
      }
      start_controllers.push_back(l_arm_controller_name_);
    }
    if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
      if(!right_running_controller.empty()) {
        stop_controllers.push_back(right_running_controller);
      }
      start_controllers.push_back(r_arm_controller_name_);
    }
  }
  switchControllers(start_controllers, stop_controllers);
  if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
    left_arm_control_mode_ = mode;
  }
  if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
    right_arm_control_mode_ = mode;
  }
}

void GeneralCommander::sendHeadCommand(double req_pan, double req_tilt) {
  if(!control_head_) return;
  if(head_control_mode_ != HEAD_JOYSTICK) {
    return;
  }
  //TODO - correct
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
  traj.joint_names.push_back("head_pan_joint");
  traj.joint_names.push_back("head_tilt_joint");
  traj.points.resize(1);
  traj.points[0].positions.push_back(req_pan);
  traj.points[0].velocities.push_back(0.0);//req_pan_vel);
  traj.points[0].positions.push_back(req_tilt);
  traj.points[0].velocities.push_back(0.0);//req_tilt_vel);
  traj.points[0].time_from_start = ros::Duration(0.1);
  //ROS_INFO_STREAM("Publishing " << req_pan << " " << req_tilt);
  head_pub_.publish(traj);
}

void GeneralCommander::sendHeadTrackCommand() {
  if(!control_head_) return;
  if(head_control_mode_ != HEAD_TRACK_LEFT_HAND &&
     head_control_mode_ != HEAD_TRACK_RIGHT_HAND) {
    return;
  }

  //the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;
  
  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  if(head_control_mode_ == HEAD_TRACK_LEFT_HAND) {
    point.header.frame_id = LEFT_HAND_LINK_TO_TRACK;
  } else {
    point.header.frame_id = RIGHT_HAND_LINK_TO_TRACK;
  }
  point.point.x = 0.0; 
  point.point.y = 0.0; 
  point.point.z = 0.0;
  goal.target = point;
  
  //we are pointing the high-def camera frame 
  //(pointing_axis defaults to X-axis)
  goal.pointing_frame = "high_def_frame";
  
  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.1);

  //and go no faster than 1 rad/s
  goal.max_velocity = MAX_HEAD_TRACK_SPEED;

  //send the goal
  head_track_hand_client_->sendGoal(goal);

  //TODO - preempty going to be problematic?

}

void GeneralCommander::sendGripperCommand(WhichArm which, bool close) {
  if(!control_rarm_ && !control_larm_) return;
  if(!control_rarm_ && which == ARMS_RIGHT) return;
  if(!control_larm_ && which == ARMS_LEFT) return;
  if(which == ARMS_RIGHT || which == ARMS_BOTH) {
    pr2_controllers_msgs::Pr2GripperCommandGoal com;
    if(close) {
      com.command.position = GRIPPER_CLOSE_POSITION;
      com.command.max_effort = GRIPPER_CLOSE_MAX_EFFORT;
    } else {
      com.command.position = GRIPPER_OPEN_POSITION;
      com.command.max_effort = GRIPPER_OPEN_MAX_EFFORT;
    }
    right_gripper_client_->sendGoal(com);
    right_gripper_client_->waitForResult(ros::Duration(5.0));
    if(right_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_DEBUG("Right gripper command succeeded");
    else
      ROS_WARN("Right gripper command failed");
  }
  if(which == ARMS_LEFT || which == ARMS_BOTH) {
    pr2_controllers_msgs::Pr2GripperCommandGoal com;
    if(close) {
      com.command.position = GRIPPER_CLOSE_POSITION;
      com.command.max_effort = GRIPPER_CLOSE_MAX_EFFORT;
    } else {
      com.command.position = GRIPPER_OPEN_POSITION;
      com.command.max_effort = GRIPPER_OPEN_MAX_EFFORT;
    }
    left_gripper_client_->sendGoal(com);
    left_gripper_client_->waitForResult(ros::Duration(5.0));
    if(left_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_DEBUG("Left gripper command succeeded");
    else
      ROS_WARN("Left gripper command failed");
  }
  
}

void GeneralCommander::sendTorsoCommand(double pos, double vel) {
  if(!control_body_) return;
  //only do this if we are commanding some velocity and not transitioning
  if(fabs(vel) < .0001 && fabs(last_torso_vel_ < .0001)) {
    //return;
  }
  last_torso_vel_ = vel;
  //ROS_INFO_STREAM("Torso pos " << pos << " vel " << vel);
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
  traj.joint_names.push_back("torso_lift_joint");
  traj.points.resize(1);
  traj.points[0].positions.push_back(pos);
  traj.points[0].velocities.push_back(vel);
  traj.points[0].time_from_start = ros::Duration(0.25);
  torso_pub_.publish(traj);
}

void GeneralCommander::sendBaseCommand(double vx, double vy, double vw) {
  if(!control_body_) return;
  geometry_msgs::Twist cmd;

  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = vw;
  base_pub_.publish(cmd);
}

void GeneralCommander::switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers) {
  pr2_mechanism_msgs::SwitchController::Request req;
  pr2_mechanism_msgs::SwitchController::Response res;
  req.start_controllers = start_controllers;
  req.stop_controllers = stop_controllers;
  for(std::vector<std::string>::const_iterator it = start_controllers.begin();
      it != start_controllers.end();
      it++) {
    ROS_DEBUG_STREAM("Trying to start controller " << (*it));
  }
  for(std::vector<std::string>::const_iterator it = stop_controllers.begin();
      it != stop_controllers.end();
      it++) {
    ROS_DEBUG_STREAM("Trying to stop controller " << (*it));
  }
  req.strictness =  pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
  if(!switch_controllers_service_.call(req,res)) {
    ROS_WARN("Call to switch controllers failed entirely");
  } 
  if(res.ok != true) {
    ROS_WARN("Call to switch controllers reports not ok");
  }
}

void GeneralCommander::sendWristVelCommands(double right_wrist_vel, double left_wrist_vel, double hz) {
  
  clampDesiredArmPositionsToActual(.05);
  
  if(control_rarm_) {
    if(abs(right_wrist_vel) > .01) {
      if((ros::Time::now()-last_right_wrist_goal_stamp_).toSec() > .5) {
        clampDesiredArmPositionsToActual(0.0);
      }
      last_right_wrist_goal_stamp_ = ros::Time::now();
      
      //our goal variable
      pr2_controllers_msgs::JointTrajectoryGoal right_goal;
      composeWristRotGoal("r", right_goal, right_des_joint_states_ ,right_wrist_vel, hz);
      right_arm_traj_pub_.publish(right_goal.trajectory);
      //right_arm_trajectory_client_->sendGoal(right_goal);
    }
  }
  if(control_larm_) {
    if(abs(left_wrist_vel) > .01) {
      if((ros::Time::now()-last_left_wrist_goal_stamp_).toSec() > .5) {
        clampDesiredArmPositionsToActual(0.0);
      }
      last_left_wrist_goal_stamp_ = ros::Time::now();
      //our goal variable
      pr2_controllers_msgs::JointTrajectoryGoal left_goal;
      composeWristRotGoal("l", left_goal, left_des_joint_states_, left_wrist_vel, hz);
      left_arm_traj_pub_.publish(left_goal.trajectory);
      //left_arm_trajectory_client_->sendGoal(left_goal);
    }
  }
}

void GeneralCommander::composeWristRotGoal(const std::string pref, pr2_controllers_msgs::JointTrajectoryGoal& goal, 
                                         std::vector<double>& des_joints, 
                                         double des_vel, double hz) const {

  double trajectory_duration = .2;

  std::vector<std::string> joint_names;

  // First, the joint names, which apply to all waypoints
  joint_names.push_back(pref+"_"+"shoulder_pan_joint");
  joint_names.push_back(pref+"_"+"shoulder_lift_joint");
  joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
  joint_names.push_back(pref+"_"+"elbow_flex_joint");
  joint_names.push_back(pref+"_"+"forearm_roll_joint");
  joint_names.push_back(pref+"_"+"wrist_flex_joint");
  joint_names.push_back(pref+"_"+"wrist_roll_joint");
  
  double end_pos_diff = (trajectory_duration)*des_vel;
  double cur_pos_diff = (1.0/hz)*des_vel;

  goal.trajectory.joint_names=joint_names;
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions = des_joints;

  goal.trajectory.points[0].velocities.resize(7,0.0);
  //goal.trajectory.points[0].velocities[6] = des_vel;
 
  goal.trajectory.points[0].positions[6] += end_pos_diff;
  des_joints[6] += cur_pos_diff;
  //ROS_INFO_STREAM("Sending pos goal " << goal.trajectory.points[1].positions[6]);
  
  goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.030);
  goal.trajectory.points[0].time_from_start = ros::Duration(trajectory_duration);
}

void GeneralCommander::updateCurrentWristPositions() {

  if(control_rarm_) {
    moveit_msgs::GetPositionFK::Request right_fk_request;
    moveit_msgs::GetPositionFK::Response right_fk_response;
    
    right_fk_request.header.frame_id = "base_link";
    right_fk_request.fk_link_names.push_back("r_wrist_roll_link");
    std::vector<std::string> right_joint_names;
    right_joint_names.push_back("r_shoulder_pan_joint");
    right_joint_names.push_back("r_shoulder_lift_joint");
    right_joint_names.push_back("r_upper_arm_roll_joint");
    right_joint_names.push_back("r_elbow_flex_joint");
    right_joint_names.push_back("r_forearm_roll_joint");
    right_joint_names.push_back("r_wrist_flex_joint");
    right_joint_names.push_back("r_wrist_roll_joint");

    right_fk_request.robot_state.joint_state.position.resize(right_joint_names.size());
    right_fk_request.robot_state.joint_state.name = right_joint_names;
    for(unsigned int i = 0; i < right_fk_request.robot_state.joint_state.name.size(); i++) {
      bool ok = getJointPosition(right_fk_request.robot_state.joint_state.name[i],  right_fk_request.robot_state.joint_state.position[i]);
      if(!ok) {
        ROS_WARN_STREAM("No joint state yet for " << right_fk_request.robot_state.joint_state.name[i]); 
        return;
      }
    }
    if(right_arm_kinematics_forward_client_.call(right_fk_request, right_fk_response)) {
      if(right_fk_response.error_code.val == right_fk_response.error_code.SUCCESS) {
        right_wrist_roll_pose_ = right_fk_response.pose_stamped[0].pose;
      } else {
        ROS_DEBUG("Right fk not a success");
      }
    } else {
      ROS_WARN("Right fk call failed all together");
    }
  }

  if(control_larm_) {
    moveit_msgs::GetPositionFK::Request left_fk_request;
    moveit_msgs::GetPositionFK::Response left_fk_response;

    left_fk_request.header.frame_id = "base_link";
    left_fk_request.fk_link_names.push_back("l_wrist_roll_link");
    std::vector<std::string> left_joint_names;
    left_joint_names.push_back("l_shoulder_pan_joint");
    left_joint_names.push_back("l_shoulder_lift_joint");
    left_joint_names.push_back("l_upper_arm_roll_joint");
    left_joint_names.push_back("l_elbow_flex_joint");
    left_joint_names.push_back("l_forearm_roll_joint");
    left_joint_names.push_back("l_wrist_flex_joint");
    left_joint_names.push_back("l_wrist_roll_joint");

    left_fk_request.robot_state.joint_state.position.resize(left_joint_names.size());
    left_fk_request.robot_state.joint_state.name = left_joint_names;
    for(unsigned int i = 0; i < left_fk_request.robot_state.joint_state.name.size(); i++) {
      bool ok = getJointPosition(left_fk_request.robot_state.joint_state.name[i],  left_fk_request.robot_state.joint_state.position[i]);
      if(!ok) {
        ROS_WARN_STREAM("No joint state yet for " << left_fk_request.robot_state.joint_state.name[i]); 
        return;
      }
    }
    if(left_arm_kinematics_forward_client_.call(left_fk_request, left_fk_response)) {
      if(left_fk_response.error_code.val == left_fk_response.error_code.SUCCESS) {
        left_wrist_roll_pose_ = left_fk_response.pose_stamped[0].pose;
      } else {
        ROS_DEBUG("Left fk not a success");
      }
    } else {
      ROS_WARN("Left fk call failed all together");
    }
    
    //ROS_INFO_STREAM("Current x " << left_wrist_roll_pose_.position.x << " y " << left_wrist_roll_pose_.position.y << " z " << left_wrist_roll_pose_.position.z);
  } 
}

void GeneralCommander::clampDesiredArmPositionsToActual(double max_dist)  {
  
  updateCurrentWristPositions();

  double tot_distance;
  if(control_rarm_) {
    tot_distance = sqrt(pow(des_r_wrist_roll_pose_.position.x-right_wrist_roll_pose_.position.x,2.0)+
                        pow(des_r_wrist_roll_pose_.position.y-right_wrist_roll_pose_.position.y,2.0)+
                        pow(des_r_wrist_roll_pose_.position.z-right_wrist_roll_pose_.position.z,2.0)+
			pow(des_r_wrist_roll_pose_.orientation.x-right_wrist_roll_pose_.orientation.x,2.0)+
			pow(des_r_wrist_roll_pose_.orientation.y-right_wrist_roll_pose_.orientation.y,2.0)+
			pow(des_r_wrist_roll_pose_.orientation.z-right_wrist_roll_pose_.orientation.z,2.0)+
			pow(des_r_wrist_roll_pose_.orientation.w-right_wrist_roll_pose_.orientation.w,2.0));
    
    //ROS_INFO_STREAM("Cur " << right_wrist_roll_pose_.position.x << " " << right_wrist_roll_pose_.position.y << " " << right_wrist_roll_pose_.position.z
    //                << " des " << des_r_wrist_roll_pose_.position.x << " " << des_r_wrist_roll_pose_.position.y << " " << des_r_wrist_roll_pose_.position.z);
    
    //ROS_INFO_STREAM("Total distance " << tot_distance);
    
    if(tot_distance > max_dist) {
      des_r_wrist_roll_pose_ = right_wrist_roll_pose_;
      
      std::vector<std::string> joint_names;
      std::string pref = "r";
      // First, the joint names, which apply to all waypoints
      joint_names.push_back(pref+"_"+"shoulder_pan_joint");
      joint_names.push_back(pref+"_"+"shoulder_lift_joint");
      joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
      joint_names.push_back(pref+"_"+"elbow_flex_joint");
      joint_names.push_back(pref+"_"+"forearm_roll_joint");
      joint_names.push_back(pref+"_"+"wrist_flex_joint");
      joint_names.push_back(pref+"_"+"wrist_roll_joint");
      right_des_joint_states_.clear();
      for(unsigned int i = 0; i < joint_names.size(); i++) {
        double cur_pos;
        getJointPosition(joint_names[i], cur_pos);
        right_des_joint_states_.push_back(cur_pos);
      }
      //ROS_INFO("Clamping right");
    }
  }

  if(control_larm_) {
    tot_distance = sqrt(pow(des_l_wrist_roll_pose_.position.x-left_wrist_roll_pose_.position.x,2.0)+
                        pow(des_l_wrist_roll_pose_.position.y-left_wrist_roll_pose_.position.y,2.0)+
                        pow(des_l_wrist_roll_pose_.position.z-left_wrist_roll_pose_.position.z,2.0)+
			pow(des_l_wrist_roll_pose_.orientation.x-left_wrist_roll_pose_.orientation.x,2.0)+
			pow(des_l_wrist_roll_pose_.orientation.y-left_wrist_roll_pose_.orientation.y,2.0)+
			pow(des_l_wrist_roll_pose_.orientation.z-left_wrist_roll_pose_.orientation.z,2.0)+
			pow(des_l_wrist_roll_pose_.orientation.w-left_wrist_roll_pose_.orientation.w,2.0));

    
    if(tot_distance > max_dist) {
      des_l_wrist_roll_pose_ = left_wrist_roll_pose_;
      std::vector<std::string> joint_names;
      std::string pref = "l";
      // First, the joint names, which apply to all waypoints
      joint_names.push_back(pref+"_"+"shoulder_pan_joint");
      joint_names.push_back(pref+"_"+"shoulder_lift_joint");
      joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
      joint_names.push_back(pref+"_"+"elbow_flex_joint");
      joint_names.push_back(pref+"_"+"forearm_roll_joint");
      joint_names.push_back(pref+"_"+"wrist_flex_joint");
      joint_names.push_back(pref+"_"+"wrist_roll_joint");
      left_des_joint_states_.clear();
      for(unsigned int i = 0; i < joint_names.size(); i++) {
        double cur_pos;
        getJointPosition(joint_names[i], cur_pos);
        left_des_joint_states_.push_back(cur_pos);
      }
      //ROS_INFO("Clamping left");
    }
  }
}

void GeneralCommander::unnormalizeTrajectory(trajectory_msgs::JointTrajectory& traj) const {

  std::vector<double> current_values;
  std::vector<bool> wraparound;
  trajectory_msgs::JointTrajectory input_trajectory = traj;

  for (size_t i=0; i<input_trajectory.joint_names.size(); i++)
  {
    std::string name = input_trajectory.joint_names[i];
    
    double pos;
    if(!getJointPosition(name, pos)) {
      ROS_WARN_STREAM("Can't unnormalize as no current joint state for " << name);
      return;
    }

    //first waypoint is unnormalized relative to current joint states
    current_values.push_back(pos);
    
#if URDFDOM_1_0_0_API
    urdf::JointConstSharedPtr joint = robot_model_.getJoint(name);
#else
    boost::shared_ptr<const urdf::Joint> joint = robot_model_.getJoint(name);
#endif
    if (joint.get() == NULL)
    {
      ROS_ERROR("Joint name %s not found in urdf model", name.c_str());
      return;
    }
    if (joint->type == urdf::Joint::CONTINUOUS) {
      wraparound.push_back(true);
    } else {
      wraparound.push_back(false);
    }
  }

  trajectory_msgs::JointTrajectory unnormalized_trajectory = input_trajectory;
  for (size_t i=0; i<unnormalized_trajectory.points.size(); i++)
  {
    for (size_t j=0; j<unnormalized_trajectory.points[i].positions.size(); j++) 
    {
      if(!wraparound[j]){
        continue;
      }
      double current = current_values[j];
      double traj = unnormalized_trajectory.points[i].positions[j];
      while ( current - traj > M_PI ) traj += 2*M_PI;
      while ( traj - current > M_PI ) traj -= 2*M_PI;
      ROS_DEBUG("Normalizing joint %s from %f to %f", unnormalized_trajectory.joint_names.at(j).c_str(), 
                unnormalized_trajectory.points[i].positions[j], traj);
      unnormalized_trajectory.points[i].positions[j] = traj;
      //all other waypoints are unnormalized relative to the previous waypoint
      current_values[j] = traj;
    }  
  }
  traj = unnormalized_trajectory; 
}

void GeneralCommander::sendArmVelCommands(double r_x_vel, double r_y_vel, double r_z_vel, double r_roll_vel, double r_pitch_vel, double r_yaw_vel,
					  double l_x_vel, double l_y_vel, double l_z_vel, double l_roll_vel, double l_pitch_vel, double l_yaw_vel,
                                        double hz) {


  ros::Time beforeCall = ros::Time::now();

  double trajectory_duration = .2;

  //ROS_INFO_STREAM("Got vels " << r_x_vel << " " << r_y_vel << " " << r_z_vel);
  clampDesiredArmPositionsToActual(.1);

  if(control_rarm_) {
    
    if(fabs(r_x_vel) > .001 || fabs(r_y_vel) > .001 || fabs(r_z_vel) > .001 ||
       fabs(r_roll_vel) > .001 || fabs(r_pitch_vel) > .001 || fabs(r_yaw_vel) > .001) {
      
      geometry_msgs::Pose right_trajectory_endpoint = des_r_wrist_roll_pose_;
      
      double pos_diff_x = r_x_vel*(1.0/hz);//*look_ahead;
      double pos_diff_y = r_y_vel*(1.0/hz);//*look_ahead;
      double pos_diff_z = r_z_vel*(1.0/hz);//*look_ahead;

      double pos_diff_roll = r_roll_vel*(1.0/hz);//*look_ahead;
      double pos_diff_pitch = r_pitch_vel*(1.0/hz);//*look_ahead;
      double pos_diff_yaw = r_yaw_vel;//*(1.0/hz);//*look_ahead;

      tf::Quaternion endpoint_quat, des_quat;
      tf::Matrix3x3 end_rot, des_rot, diff_rot, duration_rot;
      end_rot.setRotation(tf::Quaternion(
			      right_trajectory_endpoint.orientation.x,
			      right_trajectory_endpoint.orientation.y,
			      right_trajectory_endpoint.orientation.z,
			      right_trajectory_endpoint.orientation.w));
      des_rot = end_rot;
      duration_rot.setEulerYPR(r_yaw_vel*trajectory_duration,
			       r_pitch_vel*trajectory_duration,
			       r_roll_vel*trajectory_duration);
      diff_rot.setEulerYPR(pos_diff_yaw, pos_diff_pitch, pos_diff_roll);

      (duration_rot *= end_rot).getRotation(endpoint_quat);
      (diff_rot *= des_rot).getRotation(des_quat);

      right_trajectory_endpoint.position.x += r_x_vel*trajectory_duration;
      right_trajectory_endpoint.position.y += r_y_vel*trajectory_duration;
      right_trajectory_endpoint.position.z += r_z_vel*trajectory_duration;

      right_trajectory_endpoint.orientation.x = endpoint_quat.getAxis().getX();
      right_trajectory_endpoint.orientation.y = endpoint_quat.getAxis().getY();
      right_trajectory_endpoint.orientation.z = endpoint_quat.getAxis().getZ();
      right_trajectory_endpoint.orientation.w = endpoint_quat.getW();
      
      des_r_wrist_roll_pose_.position.x += pos_diff_x;
      des_r_wrist_roll_pose_.position.y += pos_diff_y;
      des_r_wrist_roll_pose_.position.z += pos_diff_z;
      
      des_r_wrist_roll_pose_.orientation.x = des_quat.getAxis().getX();
      des_r_wrist_roll_pose_.orientation.y = des_quat.getAxis().getY();
      des_r_wrist_roll_pose_.orientation.z = des_quat.getAxis().getZ();
      des_r_wrist_roll_pose_.orientation.w = des_quat.getW();

      //ROS_INFO_STREAM("Desired " << des_r_wrist_roll_pose_.position.x << " " << des_r_wrist_roll_pose_.position.y << " " << des_r_wrist_roll_pose_.position.z);
      
      //now call ik
      
      moveit_msgs::GetPositionIK::Request  ik_req;
      moveit_msgs::GetPositionIK::Response ik_res;
      
      ik_req.ik_request.timeout = ros::Duration(0.05);
      ik_req.ik_request.ik_link_name = "r_wrist_roll_link";
      ik_req.ik_request.pose_stamped.header.frame_id = "base_link";
      ik_req.ik_request.pose_stamped.header.stamp = ros::Time::now();
      ik_req.ik_request.pose_stamped.pose = right_trajectory_endpoint;
      
      std::vector<std::string> right_joint_names;
      right_joint_names.push_back("r_shoulder_pan_joint");
      right_joint_names.push_back("r_shoulder_lift_joint");
      right_joint_names.push_back("r_upper_arm_roll_joint");
      right_joint_names.push_back("r_elbow_flex_joint");
      right_joint_names.push_back("r_forearm_roll_joint");
      right_joint_names.push_back("r_wrist_flex_joint");
      right_joint_names.push_back("r_wrist_roll_joint");
      ik_req.ik_request.robot_state.joint_state.position.resize(right_joint_names.size());
      ik_req.ik_request.robot_state.joint_state.name = right_joint_names;
      for(unsigned int i = 0; i < ik_req.ik_request.robot_state.joint_state.name.size(); i++) {
        bool ok = getJointPosition(ik_req.ik_request.robot_state.joint_state.name[i], ik_req.ik_request.robot_state.joint_state.position[i]);
        if(!ok) {
          ROS_WARN_STREAM("No joint state yet for " << ik_req.ik_request.robot_state.joint_state.name[i]); 
          return;
        } else {
          //ROS_INFO_STREAM("Setting position " << ik_req.ik_request.ik_seed_state.joint_state.position[i] << " For name " 
          //                << ik_req.ik_request.ik_seed_state.joint_state.name[i]);
        }
      }
      
      
      //otherwise not a lot to be done
      if(right_arm_kinematics_inverse_client_.call(ik_req, ik_res) && ik_res.error_code.val == ik_res.error_code.SUCCESS) {
        //ROS_INFO("Ik succeeded");
        pr2_controllers_msgs::JointTrajectoryGoal goal;
        
        std::vector<std::string> joint_names;
        std::string pref = "r";
        
        // First, the joint names, which apply to all waypoints
        joint_names.push_back(pref+"_"+"shoulder_pan_joint");
        joint_names.push_back(pref+"_"+"shoulder_lift_joint");
        joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
        joint_names.push_back(pref+"_"+"elbow_flex_joint");
        joint_names.push_back(pref+"_"+"forearm_roll_joint");
        joint_names.push_back(pref+"_"+"wrist_flex_joint");
        joint_names.push_back(pref+"_"+"wrist_roll_joint");
        
        goal.trajectory.joint_names = joint_names;
        
        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);
        
        for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
          goal.trajectory.points[0].positions.push_back(ik_res.solution.joint_state.position[i]);
          goal.trajectory.points[0].velocities.push_back(0.0);
        }      
        goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.200);
        goal.trajectory.points[0].time_from_start = ros::Duration(trajectory_duration);
        
        unnormalizeTrajectory(goal.trajectory);
        right_arm_traj_pub_.publish(goal.trajectory);
      } else {
        ROS_DEBUG_STREAM("Ik failed with response " << ik_res.error_code.val);
      }
    }
    
    ros::Time afterCall = ros::Time::now();
  }
  if(control_larm_) {
    if(fabs(l_x_vel) > .001 || fabs(l_y_vel) > .001 || fabs(l_z_vel) > .001 ||
       fabs(l_roll_vel) > .001 || fabs(l_pitch_vel) > .001 || fabs(l_yaw_vel) > .001) {

      geometry_msgs::Pose left_trajectory_endpoint = des_l_wrist_roll_pose_;

      double pos_diff_x = l_x_vel*(1.0/hz);
      double pos_diff_y = l_y_vel*(1.0/hz);
      double pos_diff_z = l_z_vel*(1.0/hz);
      double pos_diff_roll = l_roll_vel*(1.0/hz);//*look_ahead;
      double pos_diff_pitch = l_pitch_vel*(1.0/hz);//*look_ahead;
      double pos_diff_yaw = l_yaw_vel*(1.0/hz);

      tf::Quaternion endpoint_quat, des_quat;
      tf::Matrix3x3 end_rot, des_rot, diff_rot, duration_rot;
      end_rot.setRotation(tf::Quaternion(
			      left_trajectory_endpoint.orientation.x,
			      left_trajectory_endpoint.orientation.y,
			      left_trajectory_endpoint.orientation.z,
			      left_trajectory_endpoint.orientation.w));
      des_rot = end_rot;
      duration_rot.setEulerYPR(l_yaw_vel*trajectory_duration,
			       l_pitch_vel*trajectory_duration,
			       l_roll_vel*trajectory_duration);
      diff_rot.setEulerYPR(pos_diff_yaw, pos_diff_pitch, pos_diff_roll);

      (duration_rot *= end_rot).getRotation(endpoint_quat);
      (diff_rot *= des_rot).getRotation(des_quat);
    
      left_trajectory_endpoint.position.x += l_x_vel*trajectory_duration;
      left_trajectory_endpoint.position.y += l_y_vel*trajectory_duration;
      left_trajectory_endpoint.position.z += l_z_vel*trajectory_duration;

      left_trajectory_endpoint.orientation.x = endpoint_quat.getAxis().getX();
      left_trajectory_endpoint.orientation.y = endpoint_quat.getAxis().getY();
      left_trajectory_endpoint.orientation.z = endpoint_quat.getAxis().getZ();
      left_trajectory_endpoint.orientation.w = endpoint_quat.getW();

      des_l_wrist_roll_pose_.position.x += pos_diff_x;
      des_l_wrist_roll_pose_.position.y += pos_diff_y;
      des_l_wrist_roll_pose_.position.z += pos_diff_z;

      des_l_wrist_roll_pose_.orientation.x = des_quat.getAxis().getX();
      des_l_wrist_roll_pose_.orientation.y = des_quat.getAxis().getY();
      des_l_wrist_roll_pose_.orientation.z = des_quat.getAxis().getZ();
      des_l_wrist_roll_pose_.orientation.w = des_quat.getW();

      //now call ik
    
      moveit_msgs::GetPositionIK::Request  ik_req;
      moveit_msgs::GetPositionIK::Response ik_res;
    
      ik_req.ik_request.timeout = ros::Duration(0.05);
      ik_req.ik_request.ik_link_name = "l_wrist_roll_link";
      ik_req.ik_request.pose_stamped.header.frame_id = "base_link";
      ik_req.ik_request.pose_stamped.header.stamp = ros::Time::now();
      ik_req.ik_request.pose_stamped.pose = left_trajectory_endpoint;

      std::vector<std::string> left_joint_names;
      left_joint_names.push_back("l_shoulder_pan_joint");
      left_joint_names.push_back("l_shoulder_lift_joint");
      left_joint_names.push_back("l_upper_arm_roll_joint");
      left_joint_names.push_back("l_elbow_flex_joint");
      left_joint_names.push_back("l_forearm_roll_joint");
      left_joint_names.push_back("l_wrist_flex_joint");
      left_joint_names.push_back("l_wrist_roll_joint");

      ik_req.ik_request.robot_state.joint_state.position.resize(left_joint_names.size());
      ik_req.ik_request.robot_state.joint_state.name = left_joint_names;
      for(unsigned int i = 0; i < ik_req.ik_request.robot_state.joint_state.name.size(); i++) {
        bool ok = getJointPosition(ik_req.ik_request.robot_state.joint_state.name[i], ik_req.ik_request.robot_state.joint_state.position[i]);
        if(!ok) {
          ROS_WARN_STREAM("No joint state yet for " << ik_req.ik_request.robot_state.joint_state.name[i]); 
          return;
        } else {
          //ROS_INFO_STREAM("Setting position " << ik_req.ik_request.ik_seed_state.joint_state.position[i] << " For name " 
          //                << ik_req.ik_request.ik_seed_state.joint_state.name[i]);
        }
      }
    
      //otherwise not a lot to be done
      if(left_arm_kinematics_inverse_client_.call(ik_req, ik_res) && ik_res.error_code.val == ik_res.error_code.SUCCESS) {
        //ROS_INFO("Ik succeeded");
        pr2_controllers_msgs::JointTrajectoryGoal goal;
      
        std::vector<std::string> joint_names;
        std::string pref = "l";
      
        // First, the joint names, which apply to all waypoints
        joint_names.push_back(pref+"_"+"shoulder_pan_joint");
        joint_names.push_back(pref+"_"+"shoulder_lift_joint");
        joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
        joint_names.push_back(pref+"_"+"elbow_flex_joint");
        joint_names.push_back(pref+"_"+"forearm_roll_joint");
        joint_names.push_back(pref+"_"+"wrist_flex_joint");
        joint_names.push_back(pref+"_"+"wrist_roll_joint");
      
        goal.trajectory.joint_names = joint_names;

        // We will have one waypoint in this goal trajectory
        goal.trajectory.points.resize(1);
      
        for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
          goal.trajectory.points[0].positions.push_back(ik_res.solution.joint_state.position[i]);
          goal.trajectory.points[0].velocities.push_back(0.0);
        }      
        goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.200);
        goal.trajectory.points[0].time_from_start = ros::Duration(trajectory_duration);

        unnormalizeTrajectory(goal.trajectory);
        left_arm_traj_pub_.publish(goal.trajectory);
      } else {
        ROS_DEBUG_STREAM("Ik failed with response " << ik_res.error_code.val);
      }
    }
  }
}

bool GeneralCommander::moveToWalkAlongArmPose() {
  
  //not implementing for one-armed robots
  if(!control_rarm_ || !control_larm_) {
    return false;
  } 

  updateCurrentWristPositions();

  ROS_DEBUG("Attempting to set arms to walk along pose");

  std::vector<std::string> joint_names;
  std::string pref;
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  pref = "r";
  
  // First, the joint names, which apply to all waypoints
  joint_names.push_back(pref+"_"+"shoulder_pan_joint");
  joint_names.push_back(pref+"_"+"shoulder_lift_joint");
  joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
  joint_names.push_back(pref+"_"+"elbow_flex_joint");
  joint_names.push_back(pref+"_"+"forearm_roll_joint");
  joint_names.push_back(pref+"_"+"wrist_flex_joint");
  joint_names.push_back(pref+"_"+"wrist_roll_joint");
  
  goal.trajectory.joint_names = joint_names;
  
  // We will have one waypoint in this goal trajectory
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions = right_walk_along_pose_;
  
  for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
    goal.trajectory.points[0].velocities.push_back(0.0);
  }      
  
  geometry_msgs::Pose right_walk_pose = getPositionFromJointsPose(right_arm_kinematics_forward_client_,  							      
								  "r_wrist_roll_link", 
								  joint_names, 
								  right_walk_along_pose_);
  double tot_distance = sqrt(pow(right_walk_pose.position.x-right_wrist_roll_pose_.position.x,2.0)+
                             pow(right_walk_pose.position.y-right_wrist_roll_pose_.position.y,2.0)+
                             pow(right_walk_pose.position.z-right_wrist_roll_pose_.position.z,2.0));
  ROS_DEBUG_STREAM("Right dist is " << tot_distance);

  if(tot_distance > .02) {
  
    goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.030);
    goal.trajectory.points[0].time_from_start = ros::Duration(3.0);
    
    unnormalizeTrajectory(goal.trajectory);

    ROS_DEBUG("Sending right arm goal");
    
    right_arm_trajectory_client_->sendGoal(goal);
    bool finished_before_timeout = right_arm_trajectory_client_->waitForResult(ros::Duration(5.0));
    
    actionlib::SimpleClientGoalState state = right_arm_trajectory_client_->getState();
    
    if(!finished_before_timeout || state != actionlib::SimpleClientGoalState::SUCCEEDED) {
      right_arm_trajectory_client_->sendGoal(goal);
      finished_before_timeout = right_arm_trajectory_client_->waitForResult(ros::Duration(5.0));
      state = right_arm_trajectory_client_->getState();
      if(!finished_before_timeout || state != actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_WARN("Not in walk along pose");
	return false;
      }
    } else {
      ROS_DEBUG("Right arm goal successful");
    }
  } else {
    ROS_DEBUG("No need for right arm goal");
  }
  joint_names.clear();
  pref = "l";
  
  // First, the joint names, which apply to all waypoints
  joint_names.push_back(pref+"_"+"shoulder_pan_joint");
  joint_names.push_back(pref+"_"+"shoulder_lift_joint");
  joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
  joint_names.push_back(pref+"_"+"elbow_flex_joint");
  joint_names.push_back(pref+"_"+"forearm_roll_joint");
  joint_names.push_back(pref+"_"+"wrist_flex_joint");
  joint_names.push_back(pref+"_"+"wrist_roll_joint");
  
  goal.trajectory.joint_names = joint_names;
  goal.trajectory.points[0].positions = left_walk_along_pose_;
  goal.trajectory.points[0].velocities.clear();
  for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
    goal.trajectory.points[0].velocities.push_back(0.0);
  }      
 
  geometry_msgs::Pose left_walk_pose = getPositionFromJointsPose(left_arm_kinematics_forward_client_,
								 "l_wrist_roll_link",
								 joint_names,
								 left_walk_along_pose_);
  tot_distance = sqrt(pow(left_walk_pose.position.x-left_wrist_roll_pose_.position.x,2.0)+
		      pow(left_walk_pose.position.y-left_wrist_roll_pose_.position.y,2.0)+
		      pow(left_walk_pose.position.z-left_wrist_roll_pose_.position.z,2.0));
  ROS_DEBUG_STREAM("Left dist is " << tot_distance);

  if(tot_distance > .02) {

    //goal.trajectory.points[0].positions[3] = 1.8;
    
    goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.030);
    goal.trajectory.points[0].time_from_start = ros::Duration(3.0);

    unnormalizeTrajectory(goal.trajectory);

    ROS_DEBUG("Sending left arm goal");
  
    left_arm_trajectory_client_->sendGoal(goal);
    bool finished_before_timeout = left_arm_trajectory_client_->waitForResult(ros::Duration(5.0));
    
    actionlib::SimpleClientGoalState state = left_arm_trajectory_client_->getState();
    
    if(!finished_before_timeout || state != actionlib::SimpleClientGoalState::SUCCEEDED) {
      left_arm_trajectory_client_->sendGoal(goal);
      finished_before_timeout = left_arm_trajectory_client_->waitForResult(ros::Duration(5.0));
      state = left_arm_trajectory_client_->getState();
      if(!finished_before_timeout || state != actionlib::SimpleClientGoalState::SUCCEEDED) {
	ROS_WARN("Not in walk along pose");
	return false;
      }
    } else {
      ROS_DEBUG("Left arm goal successful");
    }
  } else {
    ROS_DEBUG("No need for arm goal");
  }

  //ros::Duration(1.0).sleep();

  updateCurrentWristPositions();
  walk_along_right_des_pose_ = right_wrist_roll_pose_;
  walk_along_left_des_pose_ = left_wrist_roll_pose_;
  //ROS_INFO_STREAM("walk right " << walk_along_right_des_pose_.position.x << " " << walk_along_right_des_pose_.position.y << " " << walk_along_right_des_pose_.position.z);
  //ROS_INFO_STREAM("walk left " << walk_along_left_des_pose_.position.x << " " << walk_along_left_des_pose_.position.y << " " << walk_along_left_des_pose_.position.z);

  walk_rdx_vals_.clear();
  walk_rdy_vals_.clear();
  walk_ldx_vals_.clear();
  walk_ldy_vals_.clear();

  return true;
}

bool GeneralCommander::initWalkAlong() {

  //not implementing for one-armed robots
  if(!control_rarm_ || !control_larm_) {
    return false;
  }  

  updateCurrentWristPositions();

  std::vector<std::string> joint_names;
  std::string pref = "r";
  
  // First, the joint names, which apply to all waypoints
  joint_names.push_back(pref+"_"+"shoulder_pan_joint");
  joint_names.push_back(pref+"_"+"shoulder_lift_joint");
  joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
  joint_names.push_back(pref+"_"+"elbow_flex_joint");
  joint_names.push_back(pref+"_"+"forearm_roll_joint");
  joint_names.push_back(pref+"_"+"wrist_flex_joint");
  joint_names.push_back(pref+"_"+"wrist_roll_joint");

  geometry_msgs::Pose right_walk_pose = getPositionFromJointsPose(right_arm_kinematics_forward_client_,  							      
								  "r_wrist_roll_link", 
								  joint_names, 
								  right_walk_along_pose_);
  double tot_distance = sqrt(pow(right_walk_pose.position.x-right_wrist_roll_pose_.position.x,2.0)+
                             pow(right_walk_pose.position.y-right_wrist_roll_pose_.position.y,2.0)+
                             pow(right_walk_pose.position.z-right_wrist_roll_pose_.position.z,2.0));
  ROS_DEBUG_STREAM("Right dist is " << tot_distance);
  
  if(tot_distance > .02) {
    walk_along_ok_ = false;
    return false;
  }
  
  joint_names.clear();
  pref = "l";
  
  // First, the joint names, which apply to all waypoints
  joint_names.push_back(pref+"_"+"shoulder_pan_joint");
  joint_names.push_back(pref+"_"+"shoulder_lift_joint");
  joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
  joint_names.push_back(pref+"_"+"elbow_flex_joint");
  joint_names.push_back(pref+"_"+"forearm_roll_joint");
  joint_names.push_back(pref+"_"+"wrist_flex_joint");
  joint_names.push_back(pref+"_"+"wrist_roll_joint");
  geometry_msgs::Pose left_walk_pose = getPositionFromJointsPose(left_arm_kinematics_forward_client_,
								 "l_wrist_roll_link",
								 joint_names,
                                                                 left_walk_along_pose_);
  tot_distance = sqrt(pow(left_walk_pose.position.x-left_wrist_roll_pose_.position.x,2.0)+
		      pow(left_walk_pose.position.y-left_wrist_roll_pose_.position.y,2.0)+
		      pow(left_walk_pose.position.z-left_wrist_roll_pose_.position.z,2.0));
  ROS_DEBUG_STREAM("Left dist is " << tot_distance);
  
  if(tot_distance > .02) {
    walk_along_ok_ = false;
    return false;
  }
  walk_along_ok_ = true;
  return true;
}

void GeneralCommander::updateWalkAlongAverages() {

  //not implementing for one-armed robots
  if(!control_rarm_ || !control_larm_) {
    return;
  } 
  
  if(walk_rdx_vals_.size() > WALK_BUFFER) {
    walk_rdx_vals_.pop_front();
  }
  if(walk_rdy_vals_.size() > WALK_BUFFER) {
    walk_rdy_vals_.pop_front();
  }
  if(walk_ldx_vals_.size() > WALK_BUFFER) {
    walk_ldx_vals_.pop_front();
  }
  if(walk_ldy_vals_.size() > WALK_BUFFER) {
    walk_ldy_vals_.pop_front();
  }

  updateCurrentWristPositions();

  //ROS_INFO_STREAM("Current right " << right_wrist_roll_pose_.position.x << " " << right_wrist_roll_pose_.position.y << " " << right_wrist_roll_pose_.position.z);
  //ROS_INFO_STREAM("Current left " << left_wrist_roll_pose_.position.x << " " << left_wrist_roll_pose_.position.y << " " << left_wrist_roll_pose_.position.z);

  double rdx = right_wrist_roll_pose_.position.x-walk_along_right_des_pose_.position.x;
  double rdy = right_wrist_roll_pose_.position.y-walk_along_right_des_pose_.position.y;

  double ldx = left_wrist_roll_pose_.position.x-walk_along_left_des_pose_.position.x;
  double ldy = left_wrist_roll_pose_.position.y-walk_along_left_des_pose_.position.y;

  //ROS_INFO_STREAM(rdx << " " << rdy << " " << ldx << " " << ldy);

  walk_rdx_vals_.push_back(rdx);
  walk_rdy_vals_.push_back(rdy);
  walk_ldx_vals_.push_back(ldx);
  walk_ldy_vals_.push_back(ldy);
  
}

void GeneralCommander::sendWalkAlongCommand(double thresh, 
                                          double x_dist_max, double x_speed_scale,
                                          double y_dist_max, double y_speed_scale,
                                          double rot_speed_scale) {
  //not implementing for one-armed robots
  if(!control_rarm_ || !control_larm_) {
    return;
  } 
  
  if(!walk_along_ok_) {
    return;
  }

  updateWalkAlongAverages();

  double av_rdx = calcAverage(walk_rdx_vals_);
  double av_rdy = calcAverage(walk_rdy_vals_);
  double av_ldx = calcAverage(walk_ldx_vals_);
  double av_ldy = calcAverage(walk_ldy_vals_);

  //ROS_INFO_STREAM(av_rdx << " " << av_rdy << " " << av_ldx << " " << av_ldy);

   if(fabs(av_rdx) < thresh) {
     av_rdx = 0.0;
   }
   if(fabs(av_rdy) < thresh) {
     av_rdy = 0.0;
   }
   if(fabs(av_ldx) < thresh) {
     av_ldx = 0.0;
   }
   if(fabs(av_ldy) < thresh) {
     av_ldy = 0.0;
   }
  
  double vx = 0.0;
  double vy = 0.0;
  double vw = 0.0;

 //  if(ldx < -.0001 && rdx > .0001) {
//     vw = (fabs(rdx)+fabs(ldx))*rot_scale;
//   } else if(ldx > .0001 && rdx < -.0001) {
//     vw = (fabs(rdx)+fabs(ldx))*(-rot_scale);
//   } else {
//     vx = xy_scale*(fmax(fabs(rdx), fabs(ldx)))*((rdx > .001 || ldx > .001) ? 1.0 : -1.0);
//     vy = xy_scale*(fmax(fabs(rdy), fabs(ldy)))*((rdy > .001 || ldy > .001) ? 1.0 : -1.0);
//   }
  //if(fabs(vx) > .0001 || fabs(vy) > .0001 || fabs(vw) > .0001) {
  //ROS_INFO_STREAM("Walk along sending " << vx << " " << vy << " " << vw << " scale " << xy_scale);
  //}
  //sendBaseCommand(vx, vy, vw);

  double av_x = av_rdx/2.0+av_ldx/2.0;
  double per_x = fabs(av_x)/x_dist_max;
  per_x = std::min(per_x, 1.0);
  vx = (per_x*per_x)*x_speed_scale*((av_x > 0) ? 1 : -1);

  double per_y = fabs(av_ldy/2.0)/y_dist_max;
  per_y = std::min(per_y, 1.0);
  vy = (per_y*per_y)*y_speed_scale*((av_ldy > 0) ? 1 : -1);

  //vx = xy_scale*((av_rdx+av_ldx)/2.0);//((av_rdx, fabs(av_ldx)))*((av_rdx > .001 || av_ldx > .001) ? 1.0 : -1.0);
  //vy = av_ldy*xy_scale;
  double per_rot = fabs(av_rdy/2.0)/y_dist_max;
  per_rot = std::min(per_rot, 1.0);
  vw = (per_rot*per_rot)*rot_speed_scale*((av_rdy > 0) ? 1 : -1);;

  //ROS_INFO_STREAM("Walk along sending " << vx << " " << vy << " " << vw);

  sendBaseCommand(vx, vy, vw);

  //Eric's hacking below this line
 //  vx = ldx * 7.0 + rdx * 2;
//   vy = ldy * 7.0 + rdy * 2 - rdx * 3;
//   vw = rdx * 12.0;
//   sendBaseCommand(vx, vy, vw);	
}

double GeneralCommander::calcAverage(const std::list<double>& av_list) const {
  double av = 0.0;
  for(std::list<double>::const_iterator it = av_list.begin();
      it != av_list.end();
      it++) {
    av += (*it);
  }
  av /= av_list.size();
  return av;
}

geometry_msgs::Pose GeneralCommander::getPositionFromJointsPose(ros::ServiceClient& service_client,  							   
							      std::string fk_link,
							      const std::vector<std::string>& joint_names, const std::vector<double>& joint_pos) {
  moveit_msgs::GetPositionFK::Request fk_request;
  moveit_msgs::GetPositionFK::Response fk_response;
  
  geometry_msgs::Pose ret_pose;

  fk_request.header.frame_id = "base_link";
  fk_request.fk_link_names.push_back(fk_link);
  fk_request.robot_state.joint_state.position.resize(joint_names.size());
  fk_request.robot_state.joint_state.name = joint_names;
  fk_request.robot_state.joint_state.position = joint_pos;
  if(service_client.call(fk_request, fk_response)) {
    if(fk_response.error_code.val == fk_response.error_code.SUCCESS) {
      ret_pose = fk_response.pose_stamped[0].pose;
    } else {
      ROS_DEBUG("fk not a success");
    }
  } else {
    ROS_WARN("fk call failed all together");
  }
  return ret_pose;
}

void GeneralCommander::sendHeadSequence(HeadSequence seq) {

  if(!control_head_) return;

  setHeadMode(HEAD_JOYSTICK);

  trajectory_msgs::JointTrajectory traj;

  if(seq == HEAD_NOD) {
    traj = head_nod_traj_;
  } else if(seq == HEAD_SHAKE){
    traj = head_shake_traj_;
  }    
  traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
  head_pub_.publish(traj);
}

void GeneralCommander::powerBoardCallback(const pr2_msgs::PowerBoardStateConstPtr &powerBoardState) {
  if(walk_along_ok_) {
    if(!powerBoardState->run_stop || !powerBoardState->wireless_stop) {
      ROS_DEBUG("Killing walk along due to stop");
      walk_along_ok_ = false;
    }
  }
}

void GeneralCommander::requestProsilicaImage(std::string ns) {
  if(!control_prosilica_) return;
  polled_camera::GetPolledImage::Request gpi_req;
  polled_camera::GetPolledImage::Response gpi_res;
  gpi_req.response_namespace = ns;
  if(!prosilica_polling_client_.call(gpi_req, gpi_res)) {
    ROS_WARN("Prosilica polling request failed");
  }
}

void GeneralCommander::tuckArms(WhichArm arm) {
  
  //can't tuck just one arm for now
  if(!control_rarm_ || !control_larm_) {
    return;
  }

  setArmMode(arm, ARM_POSITION_CONTROL);

  pr2_common_action_msgs::TuckArmsGoal tuck_arm_goal;
  
  if(arm == ARMS_BOTH) {
    tuck_arm_goal.tuck_right = true;
    tuck_arm_goal.tuck_left = true;
  } else {
    ROS_DEBUG("Tucking one arm not supported");
  }

  ROS_DEBUG("Sending tuck arms");

  tuck_arms_client_->sendGoalAndWait(tuck_arm_goal, ros::Duration(10.0), ros::Duration(5.0));
}

void GeneralCommander::untuckArms(WhichArm arm) {

  //can't tuck just one arm for now
  if(!control_rarm_ || !control_larm_) {
    return;
  }

  setArmMode(arm, ARM_POSITION_CONTROL);

  pr2_common_action_msgs::TuckArmsGoal tuck_arm_goal;
  
  if(arm == ARMS_BOTH) {
    tuck_arm_goal.tuck_right = false;
    tuck_arm_goal.tuck_left = false; 
  } else {
    ROS_DEBUG("Untucking one arm not supported");
  }

  ROS_DEBUG("Sending untuck arms");

  tuck_arms_client_->sendGoalAndWait(tuck_arm_goal, ros::Duration(10.0), ros::Duration(5.0));

}
