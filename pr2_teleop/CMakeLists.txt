cmake_minimum_required(VERSION 2.8.3)
project(pr2_teleop)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  pr2_controllers_msgs
  roscpp
  rospy
  std_msgs
  tf
  topic_tools
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_teleop
#  CATKIN_DEPENDS actionlib geometry_msgs pr2_controllers_msgs roscpp rospy std_msgs tf topic_tools
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_gripper src/teleop_gripper.cpp)
add_executable(teleop_pr2 src/teleop_pr2.cpp)
add_executable(teleop_pr2_keyboard src/teleop_pr2_keyboard.cpp) 

target_link_libraries(teleop_gripper
  ${catkin_LIBRARIES}
)
target_link_libraries(teleop_pr2
  ${catkin_LIBRARIES}
)
target_link_libraries(teleop_pr2_keyboard
  ${catkin_LIBRARIES}
)

install(TARGETS teleop_pr2_keyboard teleop_pr2 teleop_gripper 
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} )

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES android_lightning_pr2.png DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
