display: Teleop
description: Teleop and control a PR2.
platform: pr2
launch: pr2_teleop_app/teleop.launch
interface: pr2_teleop_app/teleop.interface
icon: pr2_teleop_app/android_lightning_pr2.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
     base_control_topic: /base_controller/command
     camera_topic: /wide_stereo/left/image_color/compressed_throttle
