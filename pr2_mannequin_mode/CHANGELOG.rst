^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_mannequin_mode
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
