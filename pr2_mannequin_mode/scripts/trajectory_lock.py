#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Vijay Pradeep


# This is a super simple script to change the trajectory setpoint to
# the current position of the joints.  This is designed to work with
# the JointSplineTrajectoryController, and has to live live in the same
# namespace as the controller, in order to link up correctly to the
# "<controller>/command" and "<controller>/state" topics".

import sys
import time
import rospy

from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

rospy.init_node("trajectory_lock")

if len(sys.argv) < 2:
    print "Using default joint bound of .08 per joint"
    joint_bounds = [.08]*10
else:
    joint_bounds = [float(x) for x in rospy.myargv()[1:]]

def callback(msg):
    global pub

    max_error = max([abs(x) for x in msg.error.positions])

    exceeded = [abs(x) > y for x,y in zip(msg.error.positions, joint_bounds)]

    print "All: %s" % "  ".join(["% .4f" % x for x in msg.error.positions] )

    if any(exceeded):
        print "Exceeded: %.4f" % max_error

        # Copy our current state into the commanded state
        cmd = trajectory_msgs.msg.JointTrajectory()
        cmd.header.stamp = msg.header.stamp
        cmd.joint_names = msg.joint_names
        cmd.points.append( trajectory_msgs.msg.JointTrajectoryPoint())
        cmd.points[0].time_from_start = rospy.Duration(.125)
        cmd.points[0].positions = msg.actual.positions
        pub.publish(cmd)
    else:
        print "Small: %.4f" % max_error

global pub
pub = rospy.Publisher("command", trajectory_msgs.msg.JointTrajectory)

rospy.Subscriber("state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, callback)

rospy.spin()
