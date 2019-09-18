cmake_minimum_required(VERSION 2.8.3)
project(pr2_app_manager)
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    DEPENDS # TODO
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS scripts/control.py scripts/install_applications.sh
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY mock_apps launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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
