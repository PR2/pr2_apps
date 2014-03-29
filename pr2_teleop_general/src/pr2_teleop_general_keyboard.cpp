/*
 * pr2_teleop_general_keyboard
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

#include "pr2_teleop_general/pr2_teleop_general_commander.h"

int kfd = 0;
struct termios cooked, raw;

class Pr2TeleopGeneralKeyboard
{
public:
  Pr2TeleopGeneralKeyboard()
  {
    gc = NULL;
  }

  ~Pr2TeleopGeneralKeyboard() {  
    if(gc != NULL) {
      delete gc;
    }
  }

  void init()
  {
    ros::NodeHandle n_local("~");
    
    // Head pan/tilt parameters
    n_local.param("max_pan", max_pan_, 2.7);
    n_local.param("max_tilt", max_tilt_, 1.4);
    n_local.param("min_tilt", min_tilt_, -0.4);
    
    n_local.param("tilt_diff", tilt_scale_, .5);
    n_local.param("pan_diff", pan_scale_, 1.2);

    n_local.param("vx_scale", vx_scale_, 0.6);
    n_local.param("vy_scale", vy_scale_, 0.6);
    n_local.param("vw_scale", vw_scale_, 0.8);

    n_local.param("torso_step", torso_step_, 0.01);
    n_local.param("min_torso", min_torso_, 0.0);
    n_local.param("max_torso", max_torso_, 1.8);

    n_local.param("arm_x_scale", arm_x_scale_, .10);
    n_local.param("arm_y_scale", arm_y_scale_, .10);
    n_local.param("arm_z_scale", arm_z_scale_, .10);
    n_local.param("arm_roll_scale", arm_roll_scale_, .10);
    n_local.param("arm_pitch_scale", arm_pitch_scale_, .10);

    n_local.param("wrist_velocity",wrist_velocity_, 4.5);
    
    n_local.param("control_prosilica", control_prosilica, true);
    
    n_local.param("control_body", control_body, true);

    n_local.param("control_larm", control_larm, true);

    n_local.param("control_rarm", control_rarm, true);

    n_local.param("control_head", control_head, true);

    std::string arm_controller_name;
    n_local.param("arm_controller_name", arm_controller_name,std::string(""));
    
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

    head_init_ = false;
    torso_init_ = false;
  }

  void sendHeadCommand(double pan_diff, double tilt_diff) {
    if(!head_init_) {
      double cur_pan_pos = 0.0;
      double cur_tilt_pos = 0.0;
      bool ok = gc->getJointPosition("head_pan_joint", cur_pan_pos);
      if(!ok) return;
      ok = gc->getJointPosition("head_tilt_joint", cur_tilt_pos);
      if(!ok) return;
      des_pan_pos_ = cur_pan_pos;
      des_tilt_pos_ = cur_tilt_pos;
      head_init_ = true;
    }
    des_pan_pos_ += pan_diff;
    des_pan_pos_ = std::min(des_pan_pos_, max_pan_);
    des_pan_pos_ = std::max(des_pan_pos_, -max_pan_);
    des_tilt_pos_ += tilt_diff;
    des_tilt_pos_ = std::min(des_tilt_pos_, max_tilt_);
    des_tilt_pos_ = std::max(des_tilt_pos_, min_tilt_);
    gc->sendHeadCommand(des_pan_pos_, des_tilt_pos_);
  }

  void sendTorsoCommand(double torso_diff) {
    // if(!torso_init_)  {
    //   double cur_torso_pos = 0.0;
    //   bool ok = gc->getJointPosition("torso_lift_joint", cur_torso_pos);
    //   if(!ok) return;
    //   des_torso_pos_ = cur_torso_pos;
    //   torso_init_ = true;
    // }
    double cur_torso_pos = 0.0;
    gc->getJointPosition("torso_lift_joint", cur_torso_pos);
    des_torso_pos_ = cur_torso_pos+torso_diff;
    des_torso_pos_ = std::min(des_torso_pos_, max_torso_);
    des_torso_pos_ = std::max(des_torso_pos_, min_torso_);
    gc->sendTorsoCommand(des_torso_pos_,.1);
  }

public:

  GeneralCommander* gc;
  
  double max_pan_, max_tilt_, min_tilt_;
  double tilt_scale_, pan_scale_;

  double des_pan_pos_;
  double des_tilt_pos_;

  bool head_init_;
  bool torso_init_;

  double min_torso_;
  double max_torso_;
  double des_torso_pos_;
  double torso_step_;

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

  double wrist_velocity_;

  bool control_body;
  bool control_head;
  bool control_larm;
  bool control_rarm;
  bool control_prosilica;
};

void spin_function() {
  ros::spin();
}

Pr2TeleopGeneralKeyboard* generalkey = NULL;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  if(generalkey) {
    delete generalkey;
  }
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_telep_general_keyboard", ros::init_options::NoSigintHandler);
  signal(SIGINT,quit);
  
  boost::thread spin_thread(boost::bind(&spin_function));

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  
  generalkey = new Pr2TeleopGeneralKeyboard();
  generalkey->init();
  char c;
  
  bool stop = false;
  while(!stop)
  {
    puts("Reading from keyboard");
    puts("---------------------------");
    if(generalkey->control_head) {
      puts("Use 'h' for head commands");
    }
    if(generalkey->control_body) {
      puts("Use 'b' for body commands");
    }
    if(generalkey->control_larm) {
      puts("Use 'l' for left arm commands");
    }
    if(generalkey->control_rarm) {
      puts("Use 'r' for right arm commands");
    }
    if(generalkey->control_larm && generalkey->control_rarm) {
      puts("Use 'a' for both arm commands");
    }
    puts("Use 'q' to quit");
      
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
      
    switch(c)
    {
    case 'h':
      {
        if(!generalkey->control_head) {
          ROS_INFO("No head control enabled");
          break;
        }
        puts("---------------------------");
        puts("Use 'z' for projector on");
        puts("Use 'x' for projector off");
        puts("Use 's' for laser slow tilt");
        puts("Use 'f' for laser fast tilt");
        puts("Use 'g' for laser tilt off");
        puts("Use 'm' to set head mannequin mode");
        puts("Use 'y' to set to keyboard head control");
        puts("Use 'i/j/k/l' in keyboard head control mode to move head");
        puts("Use 'q' to quit head mode and return to main menu");
        bool head_stop = false;

        while(!head_stop) 
        {
          // get the next event from the keyboard
          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(-1);
          }
          switch(c) {
          case 'z':
            generalkey->gc->sendProjectorStartStop(true);
            break;
          case 'x':
            generalkey->gc->sendProjectorStartStop(false);
            break;
          case 's':
            generalkey->gc->setLaserMode(GeneralCommander::LASER_TILT_SLOW);
            break;
          case 'f':
            generalkey->gc->setLaserMode(GeneralCommander::LASER_TILT_FAST);
            break;
          case 'g':
            generalkey->gc->setLaserMode(GeneralCommander::LASER_TILT_OFF);
            break;
          case 'm':
            generalkey->gc->setHeadMode(GeneralCommander::HEAD_MANNEQUIN);
            generalkey->head_init_ = true;
            break;
          case 'y':
            generalkey->gc->setHeadMode(GeneralCommander::HEAD_JOYSTICK);
            break;
          case 'i':
            if(generalkey->gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK) {
              generalkey->sendHeadCommand(0.0, -generalkey->tilt_scale_);
            }
            break;
          case 'k':
            if(generalkey->gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK) {
              generalkey->sendHeadCommand(0.0, generalkey->tilt_scale_);
            }
            break;
          case 'j':
            if(generalkey->gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK) {
              generalkey->sendHeadCommand(generalkey->pan_scale_,0.0);
            }
            break;
          case 'l':
            if(generalkey->gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK) {
              generalkey->sendHeadCommand(-generalkey->pan_scale_, 0.0);
            }
            break;
          case 'q':
            head_stop = true;
            break;
          }
        }
      }
      break;
    case 'b':
      {
        if(!generalkey->control_body) {
          ROS_INFO("No head control enabled");
          break;
        }
        puts("---------------------------");
        puts("Use 'u' for torso up");
        puts("Use 'd' for torso down");
        puts("Use 'i/k' for forward/back");
        puts("Use 'j/l' for turning left and right");
        puts("Use 'n/m' for straffing left and right");
        puts("Use 'q' to quit body mode and return to main menu");

        bool body_stop = false;

        while(!body_stop) 
        {
          // get the next event from the keyboard
          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(-1);
          }
          switch(c) {
          case 'u':
            generalkey->sendTorsoCommand(generalkey->torso_step_);
            break;
          case 'd':
            generalkey->sendTorsoCommand(-generalkey->torso_step_);
            break;
          case 'i':
            generalkey->gc->sendBaseCommand(generalkey->vx_scale_, 0.0, 0.0);
            break;
          case 'k':
            generalkey->gc->sendBaseCommand(-generalkey->vx_scale_, 0.0, 0.0);
            break;
          case 'j':
            generalkey->gc->sendBaseCommand(0.0, 0.0, generalkey->vw_scale_);
            break;
          case 'l':
            generalkey->gc->sendBaseCommand(0.0, 0.0, -generalkey->vw_scale_);
            break;
          case 'n':
            generalkey->gc->sendBaseCommand(0.0, generalkey->vy_scale_, 0.0);
            break;
          case 'm':
            generalkey->gc->sendBaseCommand(0.0, -generalkey->vy_scale_, 0.0);
            break;
          case 'q':
            body_stop = true;
            break;
          }
        }
      }
      break;
    case 'l': case 'r': case 'a':
      {
        if(c == 'l' && !generalkey->control_larm) {
          ROS_INFO("No left arm control enabled");
          break;
        }
        if(c == 'r' && !generalkey->control_rarm) {
          ROS_INFO("No right arm control enabled");
          break;
        }
        if(c == 'a' && (!generalkey->control_larm || !generalkey->control_rarm)) {
          ROS_INFO("Both arms not enabled");
          break;
        }
        GeneralCommander::WhichArm arm;
        if(c == 'l') {
          arm = GeneralCommander::ARMS_LEFT;
        } else if(c == 'r') {
          arm = GeneralCommander::ARMS_RIGHT;
        } else {
          arm = GeneralCommander::ARMS_BOTH;
        }
        puts("---------------------------");
        puts("Use 'o' for gripper open");
        puts("Use 'p' for gripper close");
        puts("Use 'r' for wrist rotate clockwise");
        puts("Use 'u' for wrist flex up");
        puts("Use 'd' for wrist flex down");
        puts("Use 't' for wrist rotate counter-clockwise");
        puts("Use 'i/k' for hand pose forward/back");
        puts("Use 'j/l' for hand pose left/right");
        puts("Use 'h/n' for hand pose up/down");
        puts("Use 'q' to quit arm mode and return to main menu");
        bool arm_stop = false;

        while(!arm_stop) 
        {
          // get the next event from the keyboard
          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(-1);
          }

          switch(c) {
          case 'o':
            generalkey->gc->sendGripperCommand(arm, false);
            break;
          case 'p':
            generalkey->gc->sendGripperCommand(arm, true);
            break;
          case 'i':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                             generalkey->arm_x_scale_, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(generalkey->arm_x_scale_,0.0,0.0,0.0,0.0,0.0,
                                             0.0, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            } else {
              generalkey->gc->sendArmVelCommands(generalkey->arm_x_scale_,0.0,0.0,0.0,0.0,0.0,
                                             generalkey->arm_x_scale_, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            }
            break;
          case 'k':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                             -generalkey->arm_x_scale_, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(-generalkey->arm_x_scale_,0.0,0.0,0.0,0.0,0.0,
                                             0.0, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            } else {
              generalkey->gc->sendArmVelCommands(-generalkey->arm_x_scale_,0.0,0.0,0.0,0.0,0.0,
                                             -generalkey->arm_x_scale_, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            }
            break;
          case 'j':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                             0.0,generalkey->arm_y_scale_,0.0,0.0,0.0,0.0,
                                             20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,generalkey->arm_y_scale_,0.0,0.0,0.0,0.0,
                                             0.0, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,generalkey->arm_y_scale_,0.0,0.0,0.0,0.0,
                                                 0.0,generalkey->arm_y_scale_, 0.0,0.0,0.0,0.0,
                                                 20.0);
            }
            break;
          case 'l':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                             0.0,-generalkey->arm_y_scale_,0.0,0.0,0.0,0.0,
                                             20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,-generalkey->arm_y_scale_,0.0,0.0,0.0,0.0,
                                             0.0, 0.0,0.0,0.0,0.0,0.0,
                                             20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,-generalkey->arm_y_scale_,0.0,0.0,0.0,0.0,
                                                 0.0,-generalkey->arm_y_scale_, 0.0,0.0,0.0,0.0,
                                                 20.0);
            }
            break;
          case 'h':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                                 0.0,0.0,generalkey->arm_z_scale_,0.0,0.0,0.0,
                                                 20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,generalkey->arm_z_scale_,0.0,0.0,0.0,
                                                 0.0, 0.0,0.0,0.0,0.0,0.0,
                                                 20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,0.0,generalkey->arm_z_scale_,0.0,0.0,0.0,
                                                 0.0,0.0,generalkey->arm_z_scale_, 0.0,0.0,0.0,
                                                 20.0);
            }
            break;
          case 'n':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                                 0.0,0.0,-generalkey->arm_z_scale_,0.0,0.0,0.0,
                                                 20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,-generalkey->arm_z_scale_,0.0,0.0,0.0,
                                                 0.0, 0.0,0.0,0.0,0.0,0.0,
                                                 20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,0.0,-generalkey->arm_z_scale_,0.0,0.0,0.0,
                                                 0.0,0.0,-generalkey->arm_z_scale_, 0.0,0.0,0.0,
                                                 20.0);
            }
            break;
          case 'r':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                                 0.0,0.0,0.0,generalkey->arm_roll_scale_,0.0,0.0,
                                                 20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,generalkey->arm_roll_scale_,0.0,0.0,
                                                 0.0, 0.0,0.0,0.0,0.0,0.0,
                                                 20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,generalkey->arm_roll_scale_,0.0,0.0,
                                                 0.0,0.0,0.0,generalkey->arm_roll_scale_,0.0,0.0,
                                                 20.0);
            }
            break;
          case 't':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                                 0.0,0.0,0.0,-generalkey->arm_roll_scale_,0.0,0.0,
                                                 20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,-generalkey->arm_roll_scale_,0.0,0.0,
                                                 0.0,0.0,0.0,0.0,0.0,0.0,
                                                 20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,-generalkey->arm_roll_scale_,0.0,0.0,
                                                 0.0,0.0,0.0,-generalkey->arm_roll_scale_,0.0,0.0,
                                                 20.0);
            }
            break;
          case 'd':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                                 0.0,0.0,0,0.0,generalkey->arm_pitch_scale_,0.0,
                                                 20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0,0.0,generalkey->arm_pitch_scale_,0.0,
                                                 0.0, 0.0,0.0,0.0,0.0,0.0,
                                                 20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0,0.0,generalkey->arm_pitch_scale_,0.0,
                                                 0.0,0.0,0,0.0,generalkey->arm_pitch_scale_,0.0,
                                                 20.0);
            }
            break;
          case 'u':
            if(arm == GeneralCommander::ARMS_LEFT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0.0,0.0,0.0,0.0,
                                                 0.0,0.0,0,0.0,-generalkey->arm_pitch_scale_,0.0,
                                                 20.0);
            } else if(arm == GeneralCommander::ARMS_RIGHT) {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0,0.0,-generalkey->arm_pitch_scale_,0.0,
                                                 0.0, 0.0,0.0,0.0,0.0,0.0,
                                                 20.0);
            } else {
              generalkey->gc->sendArmVelCommands(0.0,0.0,0,0.0,-generalkey->arm_pitch_scale_,0.0,
                                                 0.0,0.0,0,0.0,-generalkey->arm_pitch_scale_,0.0,
                                                 20.0);
            }
            break;
          case 'q':
            arm_stop = true;
            break;
          }
        }
      }
      break;
    case 'q':
      stop = true;
      break;
    default:
      ROS_INFO_STREAM("Keycode is " << c);
      break;
    }
  }
  
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  spin_thread.join();
  return(0);
}
