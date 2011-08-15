#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_position_scripts')

import rospy
import actionlib
from trajectory_msgs.msg import *



torso_pub= rospy.Publisher('torso_controller/command', JointTrajectory)
rospy.init_node('torso_up')

while not rospy.is_shutdown():
    traj = JointTrajectory()
    traj.header.stamp = rospy.get_rostime()
    traj.joint_names.append("torso_lift_joint");
    traj.points.append(JointTrajectoryPoint())
    traj.points[0].positions.append(0.0)
    traj.points[0].velocities.append(0.1)
    traj.points[0].time_from_start = rospy.Duration(0.25)
    torso_pub.publish(traj)
    rospy.sleep(rospy.Duration.from_sec(0.2))

