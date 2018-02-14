#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, 2010, Willow Garage, Inc.
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
# Author: Wim Meeussen
"""
usage: tuck_arms.py [-l ACTION] [-r ACTION] [-q]
Options:
  -l or --left   Action for left arm
  -r or --right  Action for right arm
  -q or --quit   Shut down the action client after completing action
Actions:
  t or tuck
  u or untuck

"""

from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal
import actionlib
import getopt
import rospy


def usage():
    print __doc__ % vars()
    rospy.signal_shutdown("Help requested")


def main():
    action_name = 'tuck_arms'
    quit_when_finished = False

    # check for command line arguments, and send goal to action server if
    # required
    myargs = rospy.myargv()[1:]
    if len(myargs):
        goal = TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = True
        opts, args = getopt.getopt(myargs, 'hql:r:', ['quit', 'left', 'right'])
        for arm, action in opts:
            if arm in ('-l', '--left'):
                if action in ('tuck', 't'):
                    goal.tuck_left = True
                elif action in ('untuck', 'u'):
                    goal.tuck_left = False
                else:
                    rospy.logerr('Invalid action for right arm: %s' % action)
                    rospy.signal_shutdown("ERROR")

            if arm in ('-r', '--right'):
                if action in ('tuck', 't'):
                    goal.tuck_right = True
                elif action in ('untuck', 'u'):
                    goal.tuck_right = False
                else:
                    rospy.logerr('Invalid action for left arm: %s' % action)
                    rospy.signal_shutdown("ERROR")

            if arm in ('--quit', '-q'):
                quit_when_finished = True

            if arm in ('--help', '-h'):
                usage()

        tuck_arm_client = actionlib.SimpleActionClient(action_name,
                                                       TuckArmsAction)
        rospy.logdebug('Waiting for action server to start')
        tuck_arm_client.wait_for_server(rospy.Duration())
        rospy.logdebug('Sending goal to action server')
        tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0),
                                           rospy.Duration(5.0))

        if quit_when_finished:
            rospy.signal_shutdown("Quitting")


if __name__ == '__main__':
    rospy.init_node('tuck_arms_client')
    main()
    rospy.spin()
