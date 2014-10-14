cmake_minimum_required(VERSION 2.8.3)
project(pr2_app_manager)
find_package(catkin REQUIRED COMPONENTS app_manager willow_maps)

catkin_package(
    DEPENDS app_manager willow_maps
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


install(FILES launch/whole_pr2_apps.launch launch/pr2_app_manager.launch  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/control.py scripts/install_applications.sh
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY mock_apps launch scripts  
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



