# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_position_scripts)

find_package(catkin REQUIRED COMPONENTS actionlib rospy tf pr2_controllers_msgs)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)


install(PROGRAMS scripts/head_down.py scripts/head_up.py scripts/torso_down.py scripts/torso_up.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
