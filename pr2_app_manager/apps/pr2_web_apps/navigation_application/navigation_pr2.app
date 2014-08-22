name: PR2 Navigation
package: navigation_application
launch_file: navigation_application/navigation_pr2.launch
description: Move the PR2 around Willow Garage
icon: images/screenshot.jpg
provides: [move_base, localization]
depends: [pr2_base, tuckarms]
robot: pr2

