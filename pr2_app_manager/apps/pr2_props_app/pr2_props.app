display: Props
description: Run PR2 Props
platform: pr2
launch: pr2_props_app/pr2_props_app.launch
interface: pr2_props_app/pr2_props.interface
icon: pr2_props_app/pr2props.jpg
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.pr2props.Pr2Props
   app: 
     gravityMode: 0
     camera_topic: /wide_stereo/left/image_color/compressed_throttle
