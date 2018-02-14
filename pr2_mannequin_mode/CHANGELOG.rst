^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_mannequin_mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2018-02-14)
------------------
* Merge pull request `#23 <https://github.com/pr2/pr2_apps/issues/23>`_ from 23pointsNorth/patch-2
  Add queue size to trajectory_lock script
* Merge pull request `#29 <https://github.com/pr2/pr2_apps/issues/29>`_ from k-okada/kinetic-devel
  Kinetic devel
* Merge pull request `#30 <https://github.com/pr2/pr2_apps/issues/30>`_ from k-okada/orph
  change maintainer to ROS orphaned package maintainer
* change maintainer to ROS orphaned package maintainer
* Merge branch 'hydro-devel' into kinetic-devel
* Add queue size to trajectory_lock script
  Add queue_size as to remove following error:
  `/opt/ros/indigo/share/pr2_mannequin_mode/scripts/trajectory_lock.py:77: SyntaxWarning: The publisher should be created with an explicit keyword argument 'queue_size'. Please see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers for more information.
  pub = rospy.Publisher("command", trajectory_msgs.msg.JointTrajectory)`
  Size of 10 was selected arbitrary as working.
* Contributors: Daniel Angelov, Kei Okada

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

0.5.14 (2014-11-20)
-------------------

0.5.13 (2014-11-18)
-------------------
* Files were in correcet positoin, script was referencing wrong subdir
* Mannequin mode installs
* Contributors: dash

0.5.12 (2014-11-17)
-------------------
* Updated maintainership
* Contributors: TheDash

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

0.5.7 (2014-09-17)
------------------
* Changelog
* install targets
* Contributors: TheDash

* install targets
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
* Fixed bug in CMake regarding pr2_ctrls_msgs
* fix version number
* fix to install script/launch
* catkinization of pr2_teleop and pr2_mannequin mode
  Included organizing files into launch/config directories.
* Remove mannequin mode
* Add a new icon for mannequin mode
* Add mannequin mode to the apps server.
* use namespaced versions of controllers
* Added Ubuntu platform tags to manifest
* Dropping mannequin mode d-gain in order to reduce controller buzzing
* Upping head mannequin gains for heavy beta head
* Arm controllers now come back up after killing mannequin mode, with unspawner. Trac `#3295 <https://github.com/PR2/pr2_apps/issues/3295>`_ `#3664 <https://github.com/PR2/pr2_apps/issues/3664>`_
* staging pr2_apps into tick-tock
* Contributors: Kei Okada, Laura Lindzey, TheDash, Tony Pratkanis, gerkey, kwc, pratkanis, vpradeep, wim
