#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'soccer_application'
import roslib; roslib.load_manifest(PKG)

import rospy
import actionlib
import random

from move_base_msgs.msg import *
from geometry_msgs.msg import * 

from sound_play.libsoundplay import SoundClient

goal_locations = [
 [[17.3892879486, 17.4826889038, 0.0], [0.0, 0.0, 0.565245940316, 0.824922436934]], # Green room
 [[20.4999046326, 38.2641105652, 0.0], [0.0, 0.0, 0.89201005183, 0.452015561053]], # Pool room
 [[5.93969917297, 23.8362388611, 0.0], [0.0, 0.0, 0.553747800533, 0.83268443807]], # Vision area
 [[17.3893508911, 55.273021698, 0.0], [0.0, 0.0, 0.0, 1.0]], # Near bike racks
 [[51.1425704956, 44.7499580383, 0.0], [0.0, 0.0, 0.947195067571, 0.320657923604]] # Outside white lab  
]

class Navigator:
  def __init__(self):
    #start playing a sound
    self.sound_client = SoundClient()
    self.sound_file = rospy.get_param('~file')
    if not self.sound_file:
      rospy.logerr("You didn't speicfy a sound. I'm not going to play anything... boring.")
    else:
      rospy.sleep(rospy.Duration(1.0))
      self.sound_client.startWave(self.sound_file)

    self.pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
    self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.mb_client.wait_for_server()
    self.current_goal = None

  def send_goal(self, goal):
    goal_msg = MoveBaseGoal()
    goal_msg.target_pose.header.frame_id = '/map'
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    goal_msg.target_pose.pose.position.x = goal[0][0]
    goal_msg.target_pose.pose.position.y = goal[0][1]
    goal_msg.target_pose.pose.position.z = goal[0][2]
    goal_msg.target_pose.pose.orientation.x = goal[1][0]
    goal_msg.target_pose.pose.orientation.y = goal[1][1]
    goal_msg.target_pose.pose.orientation.z = goal[1][2]
    goal_msg.target_pose.pose.orientation.w = goal[1][3]

    self.mb_client.send_goal(goal_msg, self.done_cb)
    self.current_goal = goal

  def done_cb(self, terminal_state, result):
    self.send_goal(goal_locations[random.randint(0, len(goal_locations) - 1)])

  def stop_sound(self):
    rospy.logerr("Can't stop sound until a bug is fixed")
#self.sound_client.stopWave(self.sound_file)
#rospy.sleep(rospy.Duration(1.0))


if __name__ == '__main__':
  rospy.init_node('soccer_executive')
  nav = Navigator()
  nav.send_goal(goal_locations[0])

  rospy.on_shutdown(nav.stop_sound)

  try:
    rospy.spin()
  except rospy.ROSInterruptException: pass

