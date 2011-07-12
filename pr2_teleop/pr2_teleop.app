platform: pr2
launch: pr2_teleop/teleop.launch
interface: pr2_teleop/teleop.interface
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
     base_control_topic: /base_controller/command

