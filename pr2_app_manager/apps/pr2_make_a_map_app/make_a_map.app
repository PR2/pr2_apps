display: Make A Map
description: Create a map for the PR2
platform: pr2
launch: pr2_make_a_map_app/make_a_map.launch
interface: pr2_make_a_map_app/make_a_map.interface
icon: pr2_make_a_map_app/map.jpg
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.makeamap.MakeAMap
   app: 
     base_control_topic: /base_controller/command
     camera_topic: /wide_stereo/left/image_color/compressed_throttle
     footprint_param: /footprint
     base_scan_topic: /base_scan
     base_scan_frame: /base_laser_link