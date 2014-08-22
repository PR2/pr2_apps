name: PR2 Navigation (Listen Only)
package: navigation_application
launch_file: navigation_application/navigation_pr2_listen.launch
description: Watch the PR2 navigate around Willow Garage. You can run this application in parallel with applications that move the base and/or arms.
icon: images/screenshot.jpg
provides: localization
depends: [pr2_base]
robot: pr2

