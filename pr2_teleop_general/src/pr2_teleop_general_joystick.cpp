/*
 * pr2_teleop_general_joystick
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include "pr2_teleop_general/pr2_teleop_general_commander.h"

enum JoystickLayoutMode {
  LAYOUT_NONE,
  LAYOUT_BODY,
  LAYOUT_HEAD,
  LAYOUT_RIGHT_ARM,
  LAYOUT_LEFT_ARM,
  LAYOUT_BOTH_ARMS
};

static const unsigned int BODY_LAYOUT_BUTTON = 10;
static const unsigned int RIGHT_ARM_LAYOUT_BUTTON = 9;
static const unsigned int LEFT_ARM_LAYOUT_BUTTON = 8;
static const unsigned int HEAD_LAYOUT_BUTTON = 11;

static const unsigned int HEAD_MODE_TOGGLE_BUTTON = 4;
static const unsigned int PROJECTOR_TOGGLE_BUTTON = 5;
static const unsigned int LASER_TOGGLE_BUTTON = 7;
static const unsigned int PROSILICA_POLL_BUTTON = 6;
static const unsigned int OPEN_GRIPPER_BUTTON = 15;
static const unsigned int CLOSE_GRIPPER_BUTTON = 13;
static const unsigned int ARM_MODE_TOGGLE_BUTTON = 4;
static const unsigned int ARM_POSE_BUTTON = 6;

static const unsigned int LEFT_AXIS_NUMBER = 1;
static const unsigned int RIGHT_AXIS_NUMBER = 1;

static const unsigned int TORSO_UP_BUTTON = 12;
static const unsigned int TORSO_DOWN_BUTTON = 14;
static const unsigned int SPEED_UP_BUTTON = 11;

static const unsigned int WRIST_CLOCKWISE_BUTTON = 12;
static const unsigned int WRIST_COUNTER_BUTTON = 14;

static const unsigned int VX_AXIS = 3;
static const unsigned int VY_AXIS = 2;
static const unsigned int VW_AXIS = 0;

static const unsigned int HEAD_PAN_AXIS = 0;
static const unsigned int HEAD_TILT_AXIS = 3;

static const unsigned int ARM_X_AXIS = 3;
static const unsigned int ARM_Y_AXIS = 2;
static const unsigned int ARM_Z_AXIS = 1;
static const unsigned int ARM_YAW_AXIS = 0;

static const unsigned int ARM_UNTUCK_BUTTON = 7;
static const unsigned int ARM_TUCK_BUTTON = 5;

static const unsigned int MOVE_TO_WALK_ALONG_BUTTON = 0;
static const unsigned int SET_WALK_ALONG_BUTTON = 3;

static const ros::Duration DOUBLE_TAP_TIMEOUT(.25);

class Pr2TeleopGeneralJoystick
{
public:
  Pr2TeleopGeneralJoystick(bool deadman_no_publish = false)
  { 
    gc = NULL;
  }

  void init()
  {
    des_pan_pos_ = des_tilt_pos_ = 0;
    vel_val_pan_ = vel_val_tilt_ = 0;
    
    des_torso_pos_ = 0;

    des_torso_vel_ = 0.0;
    
    ros::NodeHandle n_local("~");

    // Head pan/tilt parameters
    n_local.param("max_pan", max_pan_, 2.7);
    n_local.param("max_tilt", max_tilt_, 1.4);
    n_local.param("min_tilt", min_tilt_, -0.4);
    
    n_local.param("tilt_scale", tilt_scale_, .5);
    n_local.param("pan_scale", pan_scale_, 1.2);
    
    n_local.param("torso_step", torso_step_, 0.05);
    n_local.param("min_torso", min_torso_, 0.0);
    n_local.param("max_torso", max_torso_, 0.3);

    n_local.param("vx_scale", vx_scale_, 0.6);
    n_local.param("vy_scale", vy_scale_, 0.6);
    n_local.param("vw_scale", vw_scale_, 0.8);
    
    n_local.param("arm_x_scale", arm_x_scale_, .15);
    n_local.param("arm_y_scale", arm_y_scale_, .15);
    n_local.param("arm_z_scale", arm_z_scale_, .15);

    n_local.param("arm_roll_scale", arm_roll_scale_, -1.50);
    n_local.param("arm_pitch_scale", arm_pitch_scale_, 1.50);
    n_local.param("arm_yaw_scale", arm_yaw_scale_, 1.50);

    n_local.param("wrist_velocity",wrist_velocity_, 4.5);

    n_local.param("walk_along_x_speed_scale", walk_along_x_speed_scale_, 9.0);
    n_local.param("walk_along_y_speed_scale", walk_along_y_speed_scale_, 9.0);
    n_local.param("walk_along_w_speed_scale", walk_along_w_speed_scale_, 9.0);
    n_local.param("walk_along_thresh", walk_along_thresh_, .015);
    n_local.param("walk_along_x_dist_max", walk_along_x_dist_max_, .5);
    n_local.param("walk_along_y_dist_max", walk_along_y_dist_max_, .5);

    n_local.param("prosilica_namespace", prosilica_namespace_, std::string("prosilica_polled"));

    bool control_prosilica;
    n_local.param("control_prosilica", control_prosilica, true);
    
    bool control_body;
    n_local.param("control_body", control_body, true);

    bool control_larm;
    n_local.param("control_larm", control_larm, true);

    bool control_rarm;
    n_local.param("control_rarm", control_rarm, true);

    bool control_head;
    n_local.param("control_head", control_head, true);

    std::string arm_controller_name;
    n_local.param("arm_controller_name", arm_controller_name,std::string(""));

    ROS_DEBUG("tilt scale: %.3f rad\n", tilt_scale_);
    ROS_DEBUG("pan scale: %.3f rad\n", pan_scale_);
    
    ROS_INFO("Initing general commander");

    if(arm_controller_name.empty()) {
      gc = new GeneralCommander(control_body, 
                                control_head, 
                                control_rarm,
                                control_larm,
                                control_prosilica);
    } else {
      gc = new GeneralCommander(control_body, 
                                control_head, 
                                control_rarm,
                                control_larm,
                                control_prosilica,
                                arm_controller_name);
    }
    first_callback_ = true;
    
    head_init_ = false;
    torso_init_ = false;
    
    proj_toggle_com_ = false;
    
    walk_along_init_waiting_ = false;
    set_walk_along_mode_ = false;

    joy_sub_ = n_.subscribe("joy", 10, &Pr2TeleopGeneralJoystick::joy_cb, this);
  }


  ~Pr2TeleopGeneralJoystick() {  
    if(gc != NULL) {
      delete gc;
    }
  }

  bool buttonOkAndOn(unsigned int buttonNum, const sensor_msgs::Joy::ConstPtr& joy_msg) const {
    if(buttonNum >= joy_msg->buttons.size()) return false;
    return(joy_msg->buttons[buttonNum]);
  }

  bool axisOk(unsigned int axisNum, const sensor_msgs::Joy::ConstPtr& joy_msg) const {
    return (axisNum < joy_msg->axes.size());
  }

  bool sameValueAsLast(unsigned int button, 
                       const sensor_msgs::Joy::ConstPtr& new_msg,
                       const sensor_msgs::Joy::ConstPtr& old_msg) {
    return (buttonOkAndOn(button, new_msg) == buttonOkAndOn(button, old_msg));
  }
                       

  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    if(first_callback_) {
      last_joy_ = joy_msg;
      first_callback_ = false;
    }

    JoystickLayoutMode layout;
    
    if(buttonOkAndOn(BODY_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_BODY;
    } else if (buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON, joy_msg) && buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_BOTH_ARMS;
    } else if (buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON,joy_msg)) {
      layout = LAYOUT_RIGHT_ARM;
    } else if (buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_LEFT_ARM;
    } else if (buttonOkAndOn(HEAD_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_HEAD;
    } else {
      layout = LAYOUT_NONE;
    }

    bool setting_walk_along_this_cycle_ = false;

    if(layout == LAYOUT_HEAD) {
      if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        ros::Time now = ros::Time::now();
        if(now-last_walk_along_time_ < DOUBLE_TAP_TIMEOUT) {
          //only matters if this is off
          if(!gc->isWalkAlongOk()) {
            set_walk_along_mode_ = true;
            setting_walk_along_this_cycle_ = true;
          }
        }
        if(gc->isWalkAlongOk()) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        } else {
          last_walk_along_time_ = now;
        }
      }
    }

    bool in_walk_along = false;
    if(gc->isWalkAlongOk()) {
      if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
	gc->turnOffWalkAlong();
	ROS_INFO("Turning off walk along");
      } else {
	vel_val_pan_ = 0.0;
	vel_val_tilt_ = 0.0;
	des_torso_vel_ = 0.0;
	des_vx_ = 0.0;
	des_vy_ = 0.0;
	des_vw_ = 0.0;
	des_right_wrist_vel_ = 0.0;
	right_arm_vx_ = 0.0;
	right_arm_vy_ = 0.0;
	right_arm_vz_ = 0.0;
	des_left_wrist_vel_ = 0.0;
	left_arm_vx_ = 0.0;
	left_arm_vy_ = 0.0;
	left_arm_vz_ = 0.0;
	right_arm_vel_roll_ = 0.0;
	right_arm_vel_pitch_ = 0.0;
	right_arm_vel_yaw_ = 0.0;
	left_arm_vel_roll_ = 0.0;
	left_arm_vel_pitch_ = 0.0;
	left_arm_vel_yaw_ = 0.0;
	in_walk_along = true;
      }
    }

    //we must be moving the arms into the mode
    if(!in_walk_along && layout == LAYOUT_HEAD && set_walk_along_mode_) {
      if(buttonOkAndOn(SET_WALK_ALONG_BUTTON, joy_msg) 
	 && !sameValueAsLast(SET_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        gc->sendHeadSequence(GeneralCommander::HEAD_SHAKE);
      }
      if(!setting_walk_along_this_cycle_ && buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        set_walk_along_mode_ = false;
        ROS_INFO("No longer waiting for set");
      }
    }

    if(!in_walk_along && layout == LAYOUT_HEAD && walk_along_init_waiting_) {
      if(buttonOkAndOn(SET_WALK_ALONG_BUTTON, joy_msg) 
         && !sameValueAsLast(SET_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        bool ok = gc->initWalkAlong();
        if(!ok) {
          gc->sendHeadSequence(GeneralCommander::HEAD_SHAKE);
        } else {
          gc->sendHeadSequence(GeneralCommander::HEAD_NOD);
          ROS_INFO("Should be in walk along");
          walk_along_init_waiting_ = false;
        }
      }
      if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        walk_along_init_waiting_ = false;
        ROS_INFO("No longer waiting for init");
      }
    }
    if(layout == LAYOUT_RIGHT_ARM || layout == LAYOUT_LEFT_ARM || layout == LAYOUT_BOTH_ARMS) {
      if(buttonOkAndOn(CLOSE_GRIPPER_BUTTON, joy_msg) 
	 && !sameValueAsLast(CLOSE_GRIPPER_BUTTON, joy_msg, last_joy_)) {
        if(layout == LAYOUT_RIGHT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, false);
        } else if(layout == LAYOUT_LEFT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_LEFT, false);
        } else {
          gc->sendGripperCommand(GeneralCommander::ARMS_BOTH, false);
        }
      }
      if(buttonOkAndOn(OPEN_GRIPPER_BUTTON, joy_msg) 
	 && !sameValueAsLast(OPEN_GRIPPER_BUTTON, joy_msg, last_joy_)) {
        if(layout == LAYOUT_RIGHT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, true);
        } else if(layout == LAYOUT_LEFT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_LEFT, true);
        } else {
          gc->sendGripperCommand(GeneralCommander::ARMS_BOTH, true);
        }
      }
    }

    if(!in_walk_along && layout == LAYOUT_HEAD) {

      if(buttonOkAndOn(PROJECTOR_TOGGLE_BUTTON, joy_msg) 
	 && !sameValueAsLast(PROJECTOR_TOGGLE_BUTTON,joy_msg, last_joy_)) {
        proj_toggle_com_ = !proj_toggle_com_;
        gc->sendProjectorStartStop(proj_toggle_com_);
      }
      
      if(buttonOkAndOn(PROSILICA_POLL_BUTTON, joy_msg) 
	 && !sameValueAsLast(PROSILICA_POLL_BUTTON,joy_msg, last_joy_)) {
        gc->requestProsilicaImage(prosilica_namespace_);
      }

      if(buttonOkAndOn(HEAD_MODE_TOGGLE_BUTTON, joy_msg) 
	 && !sameValueAsLast(HEAD_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
        
        if(gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK) {
          ROS_DEBUG("Head mode to left");
          gc->setHeadMode(GeneralCommander::HEAD_TRACK_LEFT_HAND);
        } else if(gc->getHeadMode() == GeneralCommander::HEAD_TRACK_LEFT_HAND) {
          gc->setHeadMode(GeneralCommander::HEAD_TRACK_RIGHT_HAND);
          ROS_DEBUG("Head mode to right");
        } else if(gc->getHeadMode() == GeneralCommander::HEAD_TRACK_RIGHT_HAND){
          gc->setHeadMode(GeneralCommander::HEAD_MANNEQUIN);
          ROS_DEBUG("Head mode to mannequin");
        } else {
          ROS_DEBUG("Head mode to joystick");
          head_init_ = false;
          gc->setHeadMode(GeneralCommander::HEAD_JOYSTICK);
        }
      }

      if(buttonOkAndOn(LASER_TOGGLE_BUTTON, joy_msg) 
	 && !sameValueAsLast(LASER_TOGGLE_BUTTON, joy_msg, last_joy_)) {
        if(gc->getLaserMode() == GeneralCommander::LASER_TILT_OFF) {
          gc->setLaserMode(GeneralCommander::LASER_TILT_SLOW);
        } else if(gc->getLaserMode() == GeneralCommander::LASER_TILT_SLOW) {
          gc->setLaserMode(GeneralCommander::LASER_TILT_FAST);
        } else {
          gc->setLaserMode(GeneralCommander::LASER_TILT_OFF);
        }
      }

      if(axisOk(HEAD_PAN_AXIS, joy_msg))
      {
        vel_val_pan_ = joy_msg->axes[HEAD_PAN_AXIS] * pan_scale_;
      }
      
      if(axisOk(HEAD_TILT_AXIS, joy_msg))
      {
        vel_val_tilt_ = joy_msg->axes[HEAD_TILT_AXIS] * tilt_scale_;
      }   
    } else {
      vel_val_pan_ = 0.0;
      vel_val_tilt_ = 0.0;
    }

    if(!in_walk_along && layout == LAYOUT_BODY) {
      bool down = buttonOkAndOn(TORSO_DOWN_BUTTON, joy_msg);
      bool up = buttonOkAndOn(TORSO_UP_BUTTON, joy_msg);
      bool speedup = buttonOkAndOn(SPEED_UP_BUTTON, joy_msg);
      bool unsafe = buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON, joy_msg); // for jsk_pr2_startup
      
      //ROS_INFO_STREAM("Down is " << down);
      //ROS_INFO_STREAM("Up is " << up);
      
      if(down && !up) {
        des_torso_vel_ = -torso_step_;
      } else if(!down && up) {
        des_torso_vel_ = torso_step_;
      } else {
	//ROS_INFO_STREAM("Setting des vel to 0.0");
        des_torso_vel_ = 0.0;
      }
      if(axisOk(VX_AXIS, joy_msg)) {
        des_vx_ = joy_msg->axes[VX_AXIS]*vx_scale_*(unsafe?0.5:(speedup?2.0:1.0));
      } else {
        des_vx_ = 0.0;
      }
      if(axisOk(VY_AXIS, joy_msg)) {
        des_vy_ = joy_msg->axes[VY_AXIS]*vy_scale_*(unsafe?0.5:(speedup?2.0:1.0));
      } else {
        des_vy_ = 0.0;
      }
      if(axisOk(VW_AXIS, joy_msg)) {
        des_vw_ = joy_msg->axes[VW_AXIS]*vw_scale_*(unsafe?0.5:(speedup?2.0:1.0));
      } else {
        des_vw_ = 0.0;
      }
    } else {
      des_torso_vel_ = 0.0;
      des_vx_ = 0.0;
      des_vy_ = 0.0;
      des_vw_ = 0.0;
    }

    if(layout == LAYOUT_RIGHT_ARM) {
      if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
	if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_POSITION_CONTROL) {
          gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_MANNEQUIN_MODE);
        } else if (gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
          gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_NO_CONTROLLER);
        } else {
          gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_POSITION_CONTROL);
        }
      }

      if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) {
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        gc->tuckArms(GeneralCommander::ARMS_RIGHT);        
      } else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) {
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        gc->untuckArms(GeneralCommander::ARMS_RIGHT);
      } 

      if(!in_walk_along) {

        bool lookAnalog = false;
        bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
        bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
	bool rotateArm = buttonOkAndOn(ARM_POSE_BUTTON, joy_msg);
        if(rotClock && !rotCounter) {
          des_right_wrist_vel_ = wrist_velocity_;
        } else if(!rotClock && rotCounter) {
          des_right_wrist_vel_ = -wrist_velocity_;
        } else {
          des_right_wrist_vel_ = 0.0;
          lookAnalog = true;
        }

	right_arm_vx_ = 0.0;
	right_arm_vy_ = 0.0;
	right_arm_vz_ = 0.0;
	right_arm_vel_roll_ = 0.0;
	right_arm_vel_pitch_ = 0.0;
	right_arm_vel_yaw_ = 0.0;
        
        if(lookAnalog) {
          //look at analog sticks if we aren't supposed to wrist rotate
          if(axisOk(ARM_X_AXIS, joy_msg)) {
	    if(!rotateArm)
	      right_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
	    else
	      right_arm_vel_pitch_ = joy_msg->axes[ARM_X_AXIS]*arm_pitch_scale_;
          }
          if(axisOk(ARM_Y_AXIS, joy_msg)) {
	    if(!rotateArm)
	      right_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
	    else
	      right_arm_vel_roll_ = joy_msg->axes[ARM_Y_AXIS]*arm_roll_scale_;
          }
          if(axisOk(ARM_Z_AXIS, joy_msg) && (!rotateArm)) {
	    right_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
          }
          if(axisOk(ARM_YAW_AXIS, joy_msg) && rotateArm) {
	    right_arm_vel_yaw_ = joy_msg->axes[ARM_YAW_AXIS]*arm_yaw_scale_;
	  }
          //ROS_INFO_STREAM("Setting vx " << right_arm_vx_ << " " << right_arm_vy_ << " " << right_arm_vz_);
        }
      }
    } else if (layout != LAYOUT_BOTH_ARMS) {
      des_right_wrist_vel_ = 0.0;
      right_arm_vx_ = 0.0;
      right_arm_vy_ = 0.0;
      right_arm_vz_ = 0.0;
      right_arm_vel_roll_ = 0.0;
      right_arm_vel_pitch_ = 0.0;
      right_arm_vel_yaw_ = 0.0;
    }
    if(layout == LAYOUT_LEFT_ARM) {
      if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
	if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        if(gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_POSITION_CONTROL) {
          gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_MANNEQUIN_MODE);
        } else if (gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
          gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_NO_CONTROLLER);
        } else {
          gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_POSITION_CONTROL);
        }
      }

      if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) {
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        gc->tuckArms(GeneralCommander::ARMS_LEFT);        
      } else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) {
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        gc->untuckArms(GeneralCommander::ARMS_LEFT);
      } 

      if(!in_walk_along) {
        bool lookAnalog = false;
        bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
        bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
	bool rotateArm = buttonOkAndOn(ARM_POSE_BUTTON, joy_msg);
        if(rotClock && !rotCounter) {
          des_left_wrist_vel_ = wrist_velocity_;
        } else if(!rotClock && rotCounter) {
          des_left_wrist_vel_ = -wrist_velocity_;
        } else {
          des_left_wrist_vel_ = 0.0;
          lookAnalog = true;
        }

	left_arm_vx_ = 0.0;
	left_arm_vy_ = 0.0;
	left_arm_vz_ = 0.0;
	left_arm_vel_roll_ = 0.0;
	left_arm_vel_pitch_ = 0.0;
	left_arm_vel_yaw_ = 0.0;
        
        if(lookAnalog) {
          //look at analog sticks if we aren't supposed to wrist rotate
          if(axisOk(ARM_X_AXIS, joy_msg)) {
	    if(!rotateArm)
	      left_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
	    else
	      left_arm_vel_pitch_ = joy_msg->axes[ARM_X_AXIS]*arm_pitch_scale_;
          }
          if(axisOk(ARM_Y_AXIS, joy_msg)) {
	    if(!rotateArm)
	      left_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
	    else
	      left_arm_vel_roll_ = joy_msg->axes[ARM_Y_AXIS]*arm_roll_scale_;
          }
          if(axisOk(ARM_Z_AXIS, joy_msg) && (!rotateArm)) {
            left_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
          }
          if(axisOk(ARM_YAW_AXIS, joy_msg) && rotateArm) {
	    left_arm_vel_yaw_ = joy_msg->axes[ARM_YAW_AXIS]*arm_yaw_scale_;
	  }
          //ROS_INFO_STREAM("Setting vx " << left_arm_vx_ << " " << left_arm_vy_ << " " << left_arm_vz_);
        }
      }
    } else if (layout != LAYOUT_BOTH_ARMS) {
      des_left_wrist_vel_ = 0.0;
      left_arm_vx_ = 0.0;
      left_arm_vy_ = 0.0;
      left_arm_vz_ = 0.0;
      left_arm_vel_roll_ = 0.0;
      left_arm_vel_pitch_ = 0.0;
      left_arm_vel_yaw_ = 0.0;
    }
    if(layout == LAYOUT_BOTH_ARMS) {
      if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
        GeneralCommander::ArmControlMode toSend;
	if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) != gc->getArmMode(GeneralCommander::ARMS_RIGHT)) {
          toSend = GeneralCommander::ARM_POSITION_CONTROL;
        } else if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_POSITION_CONTROL) {
          toSend = GeneralCommander::ARM_MANNEQUIN_MODE;
        } else if (gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
          toSend = GeneralCommander::ARM_NO_CONTROLLER;
        } else {
          toSend = GeneralCommander::ARM_POSITION_CONTROL;
        }
        gc->setArmMode(GeneralCommander::ARMS_BOTH, toSend);
      }

      if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) {
        if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        gc->tuckArms(GeneralCommander::ARMS_BOTH);        
      } else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) {
        if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        gc->untuckArms(GeneralCommander::ARMS_BOTH);
      } 

      if(!in_walk_along) {
        bool lookAnalog = false;
        bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
        bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
        bool rotateArm = buttonOkAndOn(ARM_POSE_BUTTON, joy_msg);
        if(rotClock && !rotCounter) {
          des_left_wrist_vel_ = wrist_velocity_;
          des_right_wrist_vel_ = wrist_velocity_;
        } else if(!rotClock && rotCounter) {
          des_left_wrist_vel_ = -wrist_velocity_;
          des_right_wrist_vel_ = -wrist_velocity_;
        } else {
          des_left_wrist_vel_ = 0.0;
          des_right_wrist_vel_ = 0.0;
          lookAnalog = true;
        }

	left_arm_vx_ = 0.0;
	left_arm_vy_ = 0.0;
	left_arm_vz_ = 0.0;
	right_arm_vx_ = 0.0;
	right_arm_vy_ = 0.0;
	right_arm_vz_ = 0.0;
	left_arm_vel_roll_ = 0.0;
	left_arm_vel_pitch_ = 0.0;
	left_arm_vel_yaw_ = 0.0;
	right_arm_vel_roll_ = 0.0;
	right_arm_vel_pitch_ = 0.0;
	right_arm_vel_yaw_ = 0.0;
        
        if(lookAnalog) {
          //look at analog sticks if we aren't supposed to wrist rotate
          if(axisOk(ARM_X_AXIS, joy_msg)) {
	    if(!rotateArm) {
	      left_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
	      right_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
	    } else {
	      left_arm_vel_pitch_ = joy_msg->axes[ARM_X_AXIS]*arm_pitch_scale_;
	      right_arm_vel_pitch_ = joy_msg->axes[ARM_X_AXIS]*arm_pitch_scale_;
	    }
          }
          if(axisOk(ARM_Y_AXIS, joy_msg)) {
	    if(!rotateArm) {
	      left_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
	      right_arm_vy_ = joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
	    } else {
	      left_arm_vel_roll_ = joy_msg->axes[ARM_Y_AXIS]*arm_roll_scale_;
	      right_arm_vel_roll_ = joy_msg->axes[ARM_Y_AXIS]*arm_roll_scale_;
	    }
          }
          if(axisOk(ARM_Z_AXIS, joy_msg) && (!rotateArm)) {
            left_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
            right_arm_vz_ = joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
          }
          if(axisOk(ARM_YAW_AXIS, joy_msg) && rotateArm) {
	    left_arm_vel_yaw_ = joy_msg->axes[ARM_YAW_AXIS]*arm_yaw_scale_;
	    right_arm_vel_yaw_ = joy_msg->axes[ARM_YAW_AXIS]*arm_yaw_scale_;
	  }
        //ROS_INFO_STREAM("Setting vx " << left_arm_vx_ << " " << left_arm_vy_ << " " << left_arm_vz_);
        }
      }
    } else if (layout != LAYOUT_RIGHT_ARM && layout != LAYOUT_LEFT_ARM) {
      des_right_wrist_vel_ = 0.0;
      des_left_wrist_vel_ = 0.0;
      left_arm_vx_ = 0.0;
      left_arm_vy_ = 0.0;
      left_arm_vz_ = 0.0;
      right_arm_vx_ = 0.0;
      right_arm_vy_ = 0.0;
      right_arm_vz_ = 0.0;
      left_arm_vel_roll_ = 0.0;
      left_arm_vel_pitch_ = 0.0;
      left_arm_vel_yaw_ = 0.0;
      right_arm_vel_roll_ = 0.0;
      right_arm_vel_pitch_ = 0.0;
      right_arm_vel_yaw_ = 0.0;
    }

    joy_deadman_ = ros::Time::now();
    last_joy_ = joy_msg;
  }

  bool convertCurrentVelToDesiredTorsoPos(double hz) {
    if(!torso_init_)  {
      double cur_torso_pos = 0.0;
      bool ok = gc->getJointPosition("torso_lift_joint", cur_torso_pos);
      if(!ok) return false;
      des_torso_pos_ = cur_torso_pos;
      torso_init_ = true;
    }
    double dt = 1.0/double(hz);
    double horizon = dt*5.0;
    double cur_torso_pos = 0.0;
    gc->getJointPosition("torso_lift_joint", cur_torso_pos);
    des_torso_pos_ = cur_torso_pos+des_torso_vel_ * horizon;
    des_torso_pos_ = std::min(des_torso_pos_, max_torso_);
    des_torso_pos_ = std::max(des_torso_pos_, min_torso_);
    return true;
  }

  bool convertCurrentVelToDesiredHeadPos(double hz) {
   
    if(!head_init_)  {
      double cur_pan_pos = 0.0;
      double cur_tilt_pos = 0.0;
      bool ok = gc->getJointPosition("head_pan_joint", cur_pan_pos);
      if(!ok) return false;
      ok = gc->getJointPosition("head_tilt_joint", cur_tilt_pos);
      if(!ok) return false;
      des_pan_pos_ = cur_pan_pos;
      des_tilt_pos_ = cur_tilt_pos;
      head_init_ = true;
    }
    if(fabs(vel_val_pan_) > .0001) {
      des_pan_pos_ = des_pan_pos_ + vel_val_pan_*(1.0/hz);
      des_pan_pos_ = std::min(des_pan_pos_, max_pan_);
      des_pan_pos_ = std::max(des_pan_pos_, -max_pan_);
    }
    if(fabs(vel_val_tilt_) > .0001) {
      des_tilt_pos_ = des_tilt_pos_ - vel_val_tilt_*(1.0/hz);
      des_tilt_pos_ = std::min(des_tilt_pos_, max_tilt_);
      des_tilt_pos_ = std::max(des_tilt_pos_, min_tilt_);
    }
    //ROS_INFO_STREAM("Des pan pos " << des_pan_pos_ << " tilt " << des_tilt_pos_);
    return true;
  }

public:
  double max_pan_, max_tilt_, min_tilt_;
  int axis_pan_, axis_tilt_;
  double pan_scale_, tilt_scale_;

  double des_pan_pos_;
  double des_tilt_pos_;

  double vel_val_pan_;
  double vel_val_tilt_;

  double des_vx_;
  double des_vy_;
  double des_vw_;

  double vx_scale_;
  double vy_scale_;
  double vw_scale_;

  double arm_x_scale_;
  double arm_y_scale_;
  double arm_z_scale_;
  double arm_roll_scale_;
  double arm_pitch_scale_;
  double arm_yaw_scale_;

  double right_arm_vx_;
  double right_arm_vy_;
  double right_arm_vz_;

  double left_arm_vx_;
  double left_arm_vy_;
  double left_arm_vz_;

  double right_arm_vel_roll_;
  double right_arm_vel_pitch_;
  double right_arm_vel_yaw_;

  double left_arm_vel_roll_;
  double left_arm_vel_pitch_;
  double left_arm_vel_yaw_;

  bool head_init_;
  bool torso_init_;

  double req_torso_vel_;
  double req_torso_pos_;

  double des_torso_pos_;
  double des_torso_vel_;
  double torso_step_;
  double min_torso_;
  double max_torso_;

  double wrist_velocity_;
  double des_right_wrist_vel_;  
  double des_left_wrist_vel_;

  double walk_along_x_speed_scale_;
  double walk_along_y_speed_scale_;
  double walk_along_w_speed_scale_;
  double walk_along_thresh_;
  double walk_along_x_dist_max_;
  double walk_along_y_dist_max_;

  bool walk_along_init_waiting_;
  bool set_walk_along_mode_;

  std::string prosilica_namespace_;

  bool proj_toggle_com_; 
 
  int projector_toggle_button_;
  int tilt_toggle_button_;
  int switch_head_control_mode_button_;

  GeneralCommander* gc;

  ros::Time joy_deadman_;

  sensor_msgs::JoyConstPtr last_joy_;
  bool first_callback_;

  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;

  ros::Time last_projector_toggle_;
  ros::Time last_laser_toggle_;
  ros::Time last_head_toggle_;

  ros::Time last_walk_along_time_;

};

static const double FastHz = 100;
static const double SlowHz = 20;

void spin_function() {
  ros::spin();
}

// void quit(int sig)
// {
//   delete generaljoy;
//   ros::shutdown();
//   spin_thread.join();
//   exit(0);
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_telep_general_joystick", ros::init_options::NoSigintHandler);
  //signal(SIGINT,quit);

  boost::thread spin_thread(boost::bind(&spin_function));

  Pr2TeleopGeneralJoystick generaljoy;
  generaljoy.init();
  
  ros::Rate pub_rate(FastHz);
    
  unsigned int counter_limit = (unsigned int)(FastHz/SlowHz);

  unsigned int counter = 0;

  ros::Time beforeCall = ros::Time::now();
  ros::Time afterCall = ros::Time::now();
  while (ros::ok()) 
  {
    //ROS_INFO_STREAM("Time since last " << (ros::Time::now()-beforeCall).toSec());
    beforeCall = ros::Time::now();

    if(!generaljoy.gc->isWalkAlongOk() && !generaljoy.set_walk_along_mode_ && !generaljoy.walk_along_init_waiting_) {
      if(generaljoy.convertCurrentVelToDesiredHeadPos(FastHz)) {
        generaljoy.gc->sendHeadCommand(generaljoy.des_pan_pos_, generaljoy.des_tilt_pos_);
      } 
      generaljoy.gc->sendHeadTrackCommand();
      generaljoy.gc->sendBaseCommand(generaljoy.des_vx_, generaljoy.des_vy_, generaljoy.des_vw_);
    }
      
    if((counter % counter_limit) == 0) {
      
      counter = 0;
      
      if(generaljoy.set_walk_along_mode_) {
        bool ok = generaljoy.gc->moveToWalkAlongArmPose();
        //if we didn't get a select while moving the arms
        if(ok && generaljoy.set_walk_along_mode_) {
          ROS_INFO("Arms in walk position");
          generaljoy.walk_along_init_waiting_ = true;
        } else {
          ROS_INFO("Arms not in walk position");
        }
        generaljoy.set_walk_along_mode_ = false;
      }
      
      if(generaljoy.gc->isWalkAlongOk()) {
        
        generaljoy.gc->sendWalkAlongCommand(generaljoy.walk_along_thresh_, 
                                            generaljoy.walk_along_x_dist_max_,
                                            generaljoy.walk_along_x_speed_scale_,
                                            generaljoy.walk_along_y_dist_max_,
                                            generaljoy.walk_along_y_speed_scale_,
                                            generaljoy.walk_along_w_speed_scale_);
      } else {
        if(generaljoy.convertCurrentVelToDesiredTorsoPos(SlowHz)) {
          generaljoy.gc->sendTorsoCommand(generaljoy.des_torso_pos_, generaljoy.des_torso_vel_);
        }
        //generaljoy.gc->updateCurrentWristPositions();
        generaljoy.gc->sendWristVelCommands(generaljoy.des_right_wrist_vel_, generaljoy.des_left_wrist_vel_, SlowHz);
        
        generaljoy.gc->sendArmVelCommands(generaljoy.right_arm_vx_, generaljoy.right_arm_vy_, generaljoy.right_arm_vz_, generaljoy.right_arm_vel_roll_, generaljoy.right_arm_vel_pitch_, generaljoy.right_arm_vel_yaw_,
                                          generaljoy.left_arm_vx_, generaljoy.left_arm_vy_, generaljoy.left_arm_vz_, generaljoy.left_arm_vel_roll_, generaljoy.left_arm_vel_pitch_, generaljoy.left_arm_vel_yaw_,
                                          SlowHz);
      }
    }
    //ROS_INFO_STREAM("Everything took " << (afterCall-beforeCall).toSec());
    counter++;
    pub_rate.sleep();
  }
  
  ros::shutdown();
  spin_thread.join();
  return 0;
}
