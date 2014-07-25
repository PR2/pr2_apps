# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_tuckarm)

find_package(catkin REQUIRED COMPONENTS rospy trajectory_msgs pr2_mechanism_msgs pr2_tuck_arms_action)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy trajectory_msgs pr2_mechanism_msgs pr2_tuck_arms_action
    INCLUDE_DIRS
    LIBRARIES
)


install(DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES icon.png
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
