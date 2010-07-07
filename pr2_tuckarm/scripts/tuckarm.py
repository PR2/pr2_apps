#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Modified by Kevin Watts for two arm use

import roslib
import signal
roslib.load_manifest('pr2_tuckarm')

import rospy
import os

from trajectory_msgs.msg import *
from pr2_mechanism_msgs.msg import *
from pr2_mechanism_msgs.srv import *

import sys
import time

name_right = 'r_arm_controller'
name_left = 'l_arm_controller'
pub_right = rospy.Publisher('%s/command'%name_right, JointTrajectory, latch=True)
pub_left = rospy.Publisher('%s/command'%name_left, JointTrajectory, latch=True)

USAGE = 'tuckarm.py <arms> ; <arms> is \'(r)ight\', \'(l)eft\', or \'(b)oth\' arms'

prev_handler = None
resp =  SwitchControllerResponse()
stop_list = []

def shutdown(sig, stackframe):
  rospy.loginfo("Exiting tuckarm and bringing arm controllers back to their original state")
  resp = switch_controller([], stop_list, SwitchControllerRequest.STRICT)
  if not resp.ok:
    rospy.logerr("Could not stop arm controllers %s" %stop_list)            
  if prev_handler is not None:
    prev_handler(signal.SIGINT,None)


def start_controller(name):
  started_trying = rospy.get_rostime()
  while not rospy.is_shutdown():
    try:
      resp = list_controller()
      i = resp.controllers.index(name)

      if resp.state[i] == 'stopped':
        resp = switch_controller([name], [], SwitchControllerRequest.STRICT)
        if not resp.ok:
          rospy.logerr("Could not start %s" % name)
          sys.exit(2)
        stop_list.append(name)
      break
    except ValueError:
      if started_trying and started_trying > rospy.get_rostime() + rospy.Duration(20.0):
        rospy.logerr("The controller %s is not loaded" % name)
        started_trying = None
      time.sleep(0.1)

def go(side, positions):
  traj = JointTrajectory()
  traj.joint_names = ["%s_shoulder_pan_joint" % side, 
                      "%s_shoulder_lift_joint" % side,
                      "%s_upper_arm_roll_joint" % side,
                      "%s_elbow_flex_joint" % side, 
                      "%s_forearm_roll_joint" % side,
                      "%s_wrist_flex_joint" % side, 
                      "%s_wrist_roll_joint" % side]
  traj.points = []

  for p in positions:
    traj.points.append(JointTrajectoryPoint(positions = p[1:],
                                            velocities = [0.0] * (len(p) - 1),
                                            accelerations = [],
                                            time_from_start = rospy.Duration(p[0])))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)

  {'l': pub_left, 'r': pub_right}[side].publish(traj)



if __name__ == '__main__':
  rospy.err('This version of tuckarm is deprecated. Use tuck_arms.py instead. See the wiki at <http://www.ros.org/wiki/pr2_tuckarm>')

  if len(sys.argv) < 2:
    print USAGE
    sys.exit(-1)

  side = sys.argv[1]
  
  rospy.init_node('tuck_arms', anonymous = True)
  rospy.loginfo("Waiting for controller manager to start")
  rospy.wait_for_service('pr2_controller_manager/switch_controller')
  list_controller = rospy.ServiceProxy('pr2_controller_manager/list_controllers', ListControllers)
  switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)


  # Override rospy's signal handling.  We'll invoke rospy's handler after
  # we're done shutting down
  prev_handler = signal.getsignal(signal.SIGINT)
  signal.signal(signal.SIGINT, shutdown)

  # check if left and right controllers are loaded
  state_left = "undefined"
  state_right = "undefined"
  list_resp = list_controller()
  for (name, state) in zip(list_resp.controllers, list_resp.state):
    if name == name_left:
      state_left = state
    if name == name_right:
      state_right = state

  if side == 'l' or side == 'left': 
    rospy.loginfo("Tucking left arm")
    # tuck traj for left arm
    start_controller(name_left)
    positions = [[3.0, 0.4,0.0,0.0,-2.25,0.0,0.0,0.0],
                 [5.8, -0.05,1.31,1.38,-2.06,1.69,-2.02,2.44]]
    go('l', positions)

  elif side == 'r' or side == 'right':
    rospy.loginfo("Tucking right arm")
    # tuck traj for right arm
    start_controller(name_right)
    positions = [[3.0, -0.4,0.0,0.0,-1.57,0.0,0.0,0.0],
                 [5.0, 0.0,1.57,-1.57,-1.57,0.0,0.0,0.0]]    

    go('r', positions)

  elif side == 'b' or side == 'both':
    rospy.loginfo("Tucking both left and right arm")
    # Both arms
    start_controller(name_right)
    start_controller(name_left)
    
    positions_r = [[3.0, -0.4,0.0,0.0,-2.25,0.0,0.0,0.0],
                   [5.0, -0.01,1.35,-1.92,-1.68, 1.35,-0.18,0.31]]
    positions_l = [[3.0, 0.4,0.0,0.0,-2.25,0.0,0.0,0.0],
                   [3.8, 0.4,0.0,0.0,-2.25,0.0,0.0,0.0],
                   [5.8, -0.05,1.31,1.38,-2.06,1.69,-2.02,2.44]]
        
    go('l', positions_l)
    go('r', positions_r)
      
  else:
    print 'Unknown side! Must be l/left, r/right, or b/both!'
    print USAGE
    sys.exit(2)
  rospy.spin()
