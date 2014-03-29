/*
 * pr2_teleop_general_commander
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

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <sensor_msgs/JointState.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <geometry_msgs/Pose.h>
#include <urdf/model.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_msgs/PowerBoardState.h>

static const std::string default_arm_controller_name="arm_controller";

class GeneralCommander {

public:

  enum WhichArm {
    ARMS_LEFT,
    ARMS_RIGHT,
    ARMS_BOTH
  };

  enum ArmControlMode{
    ARM_NO_CONTROLLER,
    ARM_MANNEQUIN_MODE,
    ARM_POSITION_CONTROL
  };

  enum HeadControlMode {
    HEAD_JOYSTICK,
    HEAD_TRACK_LEFT_HAND,
    HEAD_TRACK_RIGHT_HAND,
    HEAD_MANNEQUIN
  };

  enum LaserControlMode {
    LASER_TILT_OFF,
    LASER_TILT_SLOW,
    LASER_TILT_FAST
  };

  enum HeadSequence {
    HEAD_NOD,
    HEAD_SHAKE
  };

public:

  GeneralCommander(bool control_body,
                   bool control_head,
                   bool control_rarm,
                   bool control_larm,
                   bool control_prosilica,
                   std::string arm_controller_name=default_arm_controller_name); 

  ~GeneralCommander();
  
  void setLaserMode(LaserControlMode mode);

  void setHeadMode(HeadControlMode mode);

  void setArmMode(WhichArm which, ArmControlMode mode);

  LaserControlMode getLaserMode();
  
  HeadControlMode getHeadMode();

  //returns right if they are different and both are requested
  ArmControlMode getArmMode(WhichArm which);

  void sendHeadCommand(double req_pan, double req_tilt);

  void sendProjectorStartStop(bool start);

  void sendGripperCommand(WhichArm which, bool close);

  void sendHeadTrackCommand();

  void sendTorsoCommand(double pos, double vel);

  void sendBaseCommand(double vx, double vy, double vw);

  void switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers);

  void sendWristVelCommands(double right_wrist_vel, double left_wrist_vel, double hz);

  bool getJointPosition(const std::string& name, double& pos) const;
  bool getJointVelocity(const std::string& name, double& vel) const;

  void updateCurrentWristPositions();

  void sendArmVelCommands(double r_x_vel, double r_y_vel, double r_z_vel, double r_roll_vel, double r_pitch_vel, double r_yaw_vel,
                          double l_x_vel, double l_y_vel, double l_z_vel, double l_roll_vel, double l_pitch_vel, double l_yaw_vel,
                          double hz);

  bool moveToWalkAlongArmPose();

  void sendWalkAlongCommand(double thresh, 
                            double x_dist_max, double x_speed_scale,
                            double y_dist_max, double y_speed_scale,
                            double rot_scale);

  void sendHeadSequence(HeadSequence seq);

  void requestProsilicaImage(std::string ns);

  bool initWalkAlong();

  void tuckArms(WhichArm arm);
  void untuckArms(WhichArm arm);

  bool isWalkAlongOk() {
    return walk_along_ok_;
  }

  void turnOffWalkAlong() {
    walk_along_ok_ = false;
  }

private:

  geometry_msgs::Pose getPositionFromJointsPose(ros::ServiceClient& service_client,  						
						std::string fk_link,
						const std::vector<std::string>& joint_names, const std::vector<double>& joint_pos);
  
  void updateWalkAlongAverages();

  void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState);

  void powerBoardCallback(const pr2_msgs::PowerBoardStateConstPtr &powerBoardState);

  void composeWristRotGoal(const std::string pref, pr2_controllers_msgs::JointTrajectoryGoal& goal, 
                           std::vector<double>& des_joints, double des_vel, double hz) const;

  void clampDesiredArmPositionsToActual(double max_dist);

  double calcAverage(const std::list<double>& av_list) const; 

  void unnormalizeTrajectory(trajectory_msgs::JointTrajectory& traj) const;

  ros::NodeHandle n_;

  bool control_body_;
  bool control_head_;
  bool control_rarm_;
  bool control_larm_;
  bool control_prosilica_;
  
  double laser_slow_period_;
  double laser_slow_amplitude_;
  double laser_slow_offset_;

  double laser_fast_period_;
  double laser_fast_amplitude_;
  double laser_fast_offset_;

  std::string r_arm_controller_name_;
  std::string l_arm_controller_name_;

  std::map<std::string, double> joint_state_position_map_;
  std::map<std::string, double> joint_state_velocity_map_;

  geometry_msgs::Pose right_wrist_roll_pose_, left_wrist_roll_pose_;
  geometry_msgs::Pose des_r_wrist_roll_pose_, des_l_wrist_roll_pose_;

  geometry_msgs::Pose walk_along_left_des_pose_, walk_along_right_des_pose_;
  std::vector<double> right_walk_along_pose_, left_walk_along_pose_;

  std::vector<double> right_des_joint_states_, left_des_joint_states_;

  std::list<double> walk_rdx_vals_, walk_rdy_vals_, walk_ldx_vals_, walk_ldy_vals_;
  bool walk_along_ok_;

  trajectory_msgs::JointTrajectory head_nod_traj_, head_shake_traj_;

  ros::ServiceClient tilt_laser_service_;
  ros::ServiceClient switch_controllers_service_;
  ros::ServiceClient right_arm_kinematics_solver_client_;
  ros::ServiceClient right_arm_kinematics_forward_client_;
  ros::ServiceClient right_arm_kinematics_inverse_client_;
  ros::ServiceClient left_arm_kinematics_solver_client_;
  ros::ServiceClient left_arm_kinematics_forward_client_;
  ros::ServiceClient left_arm_kinematics_inverse_client_;
  ros::ServiceClient prosilica_polling_client_;
  ros::Publisher head_pub_;
  ros::Publisher torso_pub_;
  ros::Publisher base_pub_;
  ros::Publisher right_arm_traj_pub_;
  ros::Publisher left_arm_traj_pub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber power_board_sub_;

  ros::Time last_right_wrist_goal_stamp_;  
  ros::Time last_left_wrist_goal_stamp_;

  double last_torso_vel_;
  
  //! A model of the robot to see which joints wrap around
  urdf::Model robot_model_;
  //! Flag that tells us if the robot model was initialized successfully
  bool robot_model_initialized_;

  bool status_projector_on;
  LaserControlMode laser_control_mode_;
  HeadControlMode head_control_mode_;
  ArmControlMode right_arm_control_mode_;
  ArmControlMode left_arm_control_mode_;

  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>* head_track_hand_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* right_gripper_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* left_gripper_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* right_arm_trajectory_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* left_arm_trajectory_client_;
  actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction>* tuck_arms_client_;
};
