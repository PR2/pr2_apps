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
EOF

chown -R applications ~applications/*

sudo -u applications rosinstall ~applications/ros --nobuild ~applications/install_applications.rosinstall

chown -R applications ~applications/*

#FIXME: get rid of the hard-coded path
cp ~applications/ros/pr2_apps/pr2_app_manager/scripts/control.py /usr/lib/cgi-bin
chown www-data /usr/lib/cgi-bin/control.py
chmod a+x /usr/lib/cgi-bin/control.py

rm ~applications/install_applications.rosinstall

#FIXME: set robot name correctly in the launch file.

chown -R applications ~applications/*

#FIXME: need to implement this somehow.
echo "Installation is now completed, except for rosmake"
echo "Please enter the following:"
echo "sudo su applications"
echo "source ~/ros/setup.bash"
echo "rosmake app_manager"
echo "exit"



