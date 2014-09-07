^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_teleop_general
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.6 (2014-09-07)
------------------

0.5.5 (2014-09-07)
------------------

0.5.4 (2014-09-07)
------------------

0.5.3 (2014-09-07)
------------------

0.5.2 (2014-09-07)
------------------

0.5.1 (2014-09-06)
------------------
* pr2_teleop_general now depends on moveit_msgs instead of kinematics_msgs
* Added dependency on moveit_msgs instead of kinematics_msgs
* Removed dependency on deprecated kinematics_msgs
* fix the disgnated files location in pr2_mannequin_mode
* suppress compile error because of lack of catkin_LIBRARIES and LIBRARIES
* Fixed linking of teleop_commander, it was in the wrong location
* Removed error for linking project
* Added bug fix for kinematics_msgs message dependency
* Fixed CMake and package to include kinematics_msgs
* catknize pr2_teleop_general
* migrate to hydro, bullet -> tf
* add yaw for control, see https://code.ros.org/trac/wg-ros-pkg/ticket/5118 for original ticket
* add wrist orientation control through keyboard
* port to joy in sensor_msgs
* Fixing some bugs about what is allowed during walk_along
* Not changing laser mode on start
* pr2_apps:
  manifest.xml: added cxx flags for library path
* Adding some useful launch files
* Arm controller name was wrong
* Needed arm controller names as more than a remap
* Adding a couple useful launch files that don't start ik processes
* Adding ik to joystick launch
* Increasing options of which components (head/body/arms) to control to (I hope) make it possible to use this for a head cart with only launch file changes.  Also adding a launch file to try on the head cart
* Initial commit of pr2_teleop_general, a package for controlling the robot's head, body, and arms with joystick and keyboard implementations
* Contributors: JSK applications, Kei Okada, TheDash, Wim Meeussen, gjones, hsu, wurm
