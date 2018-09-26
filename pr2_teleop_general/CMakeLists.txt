cmake_minimum_required(VERSION 2.8.3)
project(pr2_teleop_general)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs geometry_msgs sensor_msgs pr2_msgs pr2_controllers_msgs pr2_controller_manager pr2_mechanism_msgs angles urdf ps3joy pr2_common_action_msgs polled_camera moveit_msgs tf)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp actionlib actionlib_msgs geometry_msgs sensor_msgs pr2_msgs pr2_controllers_msgs pr2_controller_manager pr2_mechanism_msgs angles urdf ps3joy pr2_common_action_msgs polled_camera moveit_msgs tf 
    INCLUDE_DIRS include
    LIBRARIES pr2_teleop_general_commander
)

find_package(PkgConfig)
pkg_check_modules(URDFDOM REQUIRED urdfdom)
message(STATUS "Find URDFDOM ${URDFDOM_VERSION}")
if ("${URDFDOM_VERSION}" VERSION_EQUAL "1.0.0" OR "${URDFDOM_VERSION}" VERSION_GREATER "1.0.0")
  add_definitions(-DURDFDOM_1_0_0_API)
endif()

add_executable(pr2_teleop_general_joystick src/pr2_teleop_general_joystick.cpp)
target_link_libraries(pr2_teleop_general_joystick  ${catkin_LIBRARIES} pr2_teleop_general_commander)

add_executable(pr2_teleop_general_keyboard src/pr2_teleop_general_keyboard.cpp)
target_link_libraries(pr2_teleop_general_keyboard  ${catkin_LIBRARIES} pr2_teleop_general_commander)


add_library(pr2_teleop_general_commander src/pr2_teleop_general_commander.cpp)
add_dependencies(pr2_teleop_general_commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(pr2_teleop_general_commander  ${catkin_LIBRARIES})
add_dependencies(pr2_teleop_general_commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS pr2_teleop_general_joystick pr2_teleop_general_keyboard pr2_teleop_general_commander
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  
)

install(TARGETS pr2_teleop_general_commander
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

