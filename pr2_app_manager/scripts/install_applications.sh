#!/bin/bash

if [ "x`whoami`" != "xroot" ] ; then
    echo "Need to run as root"
    exit 1
fi

if [ -d ~applications ] ; then
    echo "User already exists"
else
    #FIXME
    echo "Create user, TODO"
    exit 1
fi

echo "Enter the robot name (e.g. pri):"
read ROBOT_NAME
if [ -z $ROBOT_NAME ] ; then
    echo "Must specify a robot name"
    exit 2
fi

cd ~applications

echo "Make ROS Directory"
mkdir -p ros

echo "Create ROS Install"

cat > ~applications/install_applications.rosinstall  <<EOF
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
- hg:
    uri: https://kforge.ros.org/mapstore/hg
    local-name: map_store
- hg:
    uri: https://kforge.ros.org/warehousewg/warehouse-hg
    local-name: warehouse
- hg:
    uri: https://kforge.ros.org/navigation/navigation
    local-name: navigation
    version: navigation-1.5.0
EOF

chown -R applications ~applications/*

sudo -u applications rosinstall ~applications/ros --nobuild ~applications/install_applications.rosinstall

chown -R applications ~applications/*

#FIXME: get rid of the hard-coded path
cp ~applications/ros/pr2_apps/pr2_app_manager/scripts/control.py /usr/lib/cgi-bin
chown www-data /usr/lib/cgi-bin/control.py
chmod a+x /usr/lib/cgi-bin/control.py

rm ~applications/install_applications.rosinstall

#Set robot name
echo "name: $ROBOT_NAME" >> ~applications/robot.yaml

chown -R applications ~applications/*

#Add two items to the CKill whitelist
if [ "x`grep su /etc/ckill/whitelist`" == "x" ] ; then
    echo "su" >> /etc/ckill/whitelist
fi                                                                                
if [ "x`grep yes /etc/ckill/whitelist`" == "x" ]; then
    echo "yes" >> /etc/ckill/whitelist
fi

su applications -c "source ~/ros/setup.bash && rosmake --rosdep-install pr2_app_manager app_manager map_store warehouse amcl navigation"



