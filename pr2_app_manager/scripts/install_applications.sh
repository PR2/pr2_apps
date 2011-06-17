#!/bin/bash

if [ -d ~applications ] ; then
    echo "User already exists"
else
    #FIXME
    echo "Create user, TODO"
    exit 1
fi

cd ~applications

echo "Make ROS Directory"
mkdir -p ros

echo "Create ROS Install"

cat > install_applications.rosinstall  <<EOF
- other:
    local-name: /opt/ros/diamondback/ros
- other:
    local-name: /opt/ros/diamondback/stacks
- svn:
    uri: https://code.ros.org/svn/ros/stacks/multimaster_experimental/trunk
    local-name: multimaster_experimental
- hg:
    uri: https://kforge.ros.org/pr2apps/pr2_apps
    local-name: pr2_apps
EOF

rosinstall ~applications/ros ~applications/install_applications.rosinstall

source ~applications/ros/setup.bash
rosmake app_manager

#FIXME: install control.py
#FIXME: need to put this script, and this launch file, in the pr2_apps stack.
#FIXME: set robot name correctly

cat > ~applications/app_man.launch <<EOF
<launch>
  <rosparam>
robot:
  name: pri1
  type: pr2
  </rosparam>

  <node pkg="app_manager" type="appmaster" name="appmaster" args="-p 11312"/>

  <node pkg="app_manager" type="app_manager" name="app_manager" args="--applist \$(find app_manager_tutorial)/applist" output="screen">
        <param name="interface_master" value="http://localhost:11312"/>
  </node>
</launch>
EOF

cat > ~applications/run.sh <<EOF
#!/bin/bash
roslaunch ~applications/app_man.launch
EOF
chmod a+x ~applications/run.sh




