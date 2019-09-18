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

# test
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()
