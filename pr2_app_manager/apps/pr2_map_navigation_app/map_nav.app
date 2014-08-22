display: Map Navigation
description: Navigation for the PR2
platform: pr2
launch: pr2_map_navigation_app/map_nav.launch
interface: pr2_map_navigation_app/map_nav.interface
icon: pr2_map_navigation_app/map.jpg
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.mapnav.MapNav
   app: 
     base_control_topic: /base_controller/command
     camera_topic: /wide_stereo/left/image_color/compressed_throttle
     footprint_param: /footprint
     base_scan_topic: /base_scan
     base_scan_frame: /base_laser_link
     path_topic: /move_base_node/NavfnROS/plan
