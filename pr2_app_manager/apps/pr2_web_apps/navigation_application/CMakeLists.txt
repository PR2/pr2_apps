# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(navigation_application)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roslib rospy webui pr2_navigation_global pr2_navigation_perception pr2_navigation_teleop pr2_machine tf laser_geometry sensor_msgs message_filters nav_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)


# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

add_executable(bin/navigation_visualizations src/navigation_visualizations.cpp)
find_package(Boost REQUIRED COMPONENTS signals system)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(bin/navigation_visualizations ${Boost_LIBRARIES})
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roslib rospy webui pr2_navigation_global pr2_navigation_perception pr2_navigation_teleop pr2_machine tf laser_geometry sensor_msgs message_filters nav_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
