#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_pan_tilt')

import rospy
import actionlib
import math
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *

import tf


def handle_android_pose(msg, listener):
    br = tf.TransformBroadcaster()
    pose = msg.pose
    tran = False
    try:
        (trans,rot) = listener.lookupTransform('/base_link', '/wide_stereo_optical_frame', rospy.Time(0))
        tran = trans
    except (tf.LookupException, tf.ConnectivityException):
        rospy.logwarn("No transform from base_link to head - application not working")
        return
    if (tran == False):
        rospy.logerr("Unknown transformation problem")
        return
    
    br.sendTransform(trans, (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(), "head_goal", "base_link")

    g = PointHeadGoal()
    g.target.header.frame_id = 'head_goal'
    g.target.point.x = 0.0
    g.target.point.y = 0.0
    g.target.point.z = -1.0
    g.min_duration = rospy.Duration(1.0)
    
    client.send_goal(g)



if __name__ == '__main__':
    rospy.init_node('move_the_head', anonymous=True)
    listener = tf.TransformListener()
    client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
    client.wait_for_server()
    rospy.Subscriber("/android/orientation", PoseStamped, handle_android_pose, listener)
    rospy.spin()
