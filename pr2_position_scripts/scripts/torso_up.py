#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_position_scripts')

import rospy
import actionlib
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPosi\
tionGoal

rospy.init_node('torso_up')
torso_action_client = actionlib.SimpleActionClient('torso_controller/position_j\
oint_action', SingleJointPositionAction);
rospy.loginfo("torso_up: waiting for torso action server")
torso_action_client.wait_for_server()
rospy.loginfo("torso_up: torso action server found")
goal = SingleJointPositionGoal()
goal.position = 0.295
rospy.loginfo("torso_up: sending command to lift torso")
torso_action_client.send_goal(goal)
torso_action_client.wait_for_result(rospy.Duration(30))

