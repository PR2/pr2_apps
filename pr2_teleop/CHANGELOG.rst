^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2018-09-26)
------------------

0.6.0 (2018-02-14)
------------------
* Merge pull request `#26 <https://github.com/pr2/pr2_apps/issues/26>`_ from TAMS-Group/pr-indigo-stop-spamming-on-slight-delays
  Only warn the user on outdated repeated messages
* Merge pull request `#27 <https://github.com/pr2/pr2_apps/issues/27>`_ from TAMS-Group/head_fix
  Using state controller feedback to update pan and tilt values of the head
* Merge pull request `#29 <https://github.com/pr2/pr2_apps/issues/29>`_ from k-okada/kinetic-devel
  Kinetic devel
* Merge pull request `#30 <https://github.com/pr2/pr2_apps/issues/30>`_ from k-okada/orph
  change maintainer to ROS orphaned package maintainer
* change maintainer to ROS orphaned package maintainer
* Merge branch 'hydro-devel' into kinetic-devel
* Only warn the user on outdated repeated messages
  dornhege's commit recently added a safety mechanism to ensure
  a properly working dead-man switch with the PS2 controller.
  However, in some setups this problem appears during normal operation
  but does not persist for long. In order to avoid spamming the log
  console, we warn the user only if the received messages are additionally
  out of date.
  The overall safety mechanism still triggers, this only affects the log.
* use desired position for head feedback
  With the actual position the head tilts down because of gravity
  when the controller is idle and head movements are less responsive.
* Pan and tilt values updated via state controller
  When controlling the head with a different controller, the head moved with a very high speed, which can be harmful to the hardware.
  It was assumed that the head was only moved via teleop, thus the position was not updated when the head was moved with a different controller.
  This was fixed by reading the actual values for pan and tilt from the state controller.
* Merge pull request `#25 <https://github.com/pr2/pr2_apps/issues/25>`_ from dornhege/indigo-devel
  Do not process the same joystick message twice.
* Do not process the same joystick message twice.
  Happens if joystick hangs on old command.
* Contributors: 2scholz, Christian Dornhege, Devon Ash, Kei Okada, v4hn

0.5.20 (2015-05-05)
-------------------

0.5.19 (2015-04-29)
-------------------
* added changelogs
* Contributors: TheDash

0.5.18 (2015-02-10)
-------------------
* fix install destination
* Contributors: Furushchev

0.5.16 (2015-02-06)
-------------------

0.5.15 (2015-01-28)
-------------------
* Added target locations to pr2_teleop libs and execs
* Added installs for pr2_teleop_general and pr2_teleop
* Added installs for pr2_teleop binaries
* Contributors: dash

0.5.14 (2014-11-20)
-------------------

0.5.13 (2014-11-18)
-------------------

0.5.12 (2014-11-17)
-------------------

0.5.11 (2014-10-20)
-------------------

0.5.10 (2014-10-17)
-------------------
* Changelogs
* Contributors: TheDash

0.5.9 (2014-10-01)
------------------

0.5.8 (2014-09-30)
------------------
* Updated maintainership
* Contributors: TheDash

0.5.7 (2014-09-17)
------------------
* Changelog
* Added install targets for pr2_teleop
* Contributors: TheDash

* Added install targets for pr2_teleop
* Contributors: TheDash

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
* fix version number
* catkinization of pr2_teleop and pr2_mannequin mode
  Included organizing files into launch/config directories.
* Merge for electric
* Add new package with scripts to adjust position of head and torso
* Remove teleop app
* Add icons for some of the apps
* port to joy in sensor_msgs
* Remove pantilt
* Add script for head teleop app
* Center the head for teleop.
* Publish video for the teleop app
* Add name and description to teleop
* Add teleop app
* Head, torso publishers don't publish if they're not advertised. `#4712 <https://github.com/PR2/pr2_apps/issues/4712>`_
* Added Ubuntu platform tags to manifest
* move parameters for teleop into yaml file. ticket 3862
* correct comment
* More useful errors
* Adding mux switching option to pr2_teleop, still needs testing on robot
* Changed default teleop_pr2 timeout to 0.5 seconds. This way it is safe by default.
* add gripper control to joystick
* Tweak to manifest description, in preparation for doc review
* staging pr2_apps into tick-tock
* Contributors: Kei Okada, Laura Lindzey, Wim Meeussen, blaise, eitan, gerkey, kwc, pratkanis, watts, wim
