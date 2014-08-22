display: PR2 Pan Tilt
description: Pan and Tilt the head of a PR2.
platform: pr2
launch: pr2_pan_tilt/pan_tilt.launch
interface: pr2_pan_tilt/pan_tilt.interface
icon: pr2_pan_tilt/pantilt.jpg
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.pantilt.PanTilt
   app: 
     gravityMode: 0
     camera_topic: /wide_stereo/left/image_color/compressed_throttle
