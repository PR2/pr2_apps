name: PR2 Recharge
package: pr2_recharge_application
launch_file: pr2_recharge_application/pr2_recharge_application.launch
#launch_file: pr2_recharge_application/dummy.launch
description: A web application to make a PR2 plug itself into a standard wall outlet.
icon: images/plug.png
provides: [pr2_recharge, move_base, arm_control, head_control]
category: Other
depends: [pr2_base]
