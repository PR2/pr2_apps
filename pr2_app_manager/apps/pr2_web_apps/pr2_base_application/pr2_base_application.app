name: PR2 Base
package: pr2_base
launch_file: pr2_base_application/pr2_base_application.launch
description: A collection of nodes that multiple apps depend on at runtime.  Eventually, many of these nodes will migrate into pr2_bringup/pr2.launch
icon: images/pluss.jpg
provides: [pr2_base, tuckarms]
category: Other
depends: 
