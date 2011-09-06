#!/bin/bash

if [ "x`whoami`" != "xroot" ] ; then
    echo "Need to run as root. Use sudo from the pr2admin user."
    exit 1
fi

USER_DIR=/u/applications

if [ -d $USER_DIR ] ; then
    echo "User already exists"
else
    echo "Adding user"
    echo -e "willow\nwillow\n" | adduser applications
    yes | su applications -c "echo \"SSH keys okay\""
fi

if [ "x`grep "rosget:" /etc/group`" == "x" ] ; then
    echo "Adding rosget group"
    addgroup rosget
fi

usermod -aG rosget applications

echo "Enter the robot name (e.g. pri):"
read ROBOT_NAME
if [ -z $ROBOT_NAME ] ; then
    echo "Must specify a robot name"
    exit 2
fi

cd $USER_DIR

echo "Make ROS Directory"
mkdir -p ros

echo "Create ROS Install"
cat > $USER_DIR/install_applications.rosinstall  <<EOF
- other:
    local-name: /opt/ros/electric/ros
- other:
    local-name: /opt/ros/electric/stacks
- svn:
    uri: https://code.ros.org/svn/ros/stacks/multimaster_experimental/trunk
    local-name: multimaster_experimental
- hg:
    uri: https://kforge.ros.org/pr2apps/pr2_apps
    local-name: pr2_apps
EOF

#Script for adding extras
# - display: Make a Map
#   app: pr2_make_a_map_app/make_a_map
# - display: Map Navigation
#   app: pr2_map_nav_app/map_nav
# - display: Map Manager
#   app: pr2_map_manager_app/map_manager
cat > /dev/null <<EOF
- hg:
    uri: https://kforge.ros.org/mapstore/hg
    local-name: map_store
- hg:
    uri: https://kforge.ros.org/pr2apps/pr2_make_a_maphg
    local-name: pr2_make_a_map_app
- hg:
    uri: https://kforge.ros.org/pr2apps/pr2_map_navhg
    local-name: pr2_map_nav_app
- hg:
    uri: https://kforge.ros.org/pr2apps/map_managerhg
    local-name: pr2_map_manager_app
EOF


echo "Run rosinstall"
chown -R applications $USER_DIR/*
rm -f $USER_DIR/ros/.rosinstall
su applications -c "rosinstall $USER_DIR/ros $USER_DIR/install_applications.rosinstall"
chown -R applications $USER_DIR/*
rm $USER_DIR/install_applications.rosinstall

echo "Set robot name"
echo "name: $ROBOT_NAME" >> $USER_DIR/robot.yaml

echo "Chown applications"
chown -R applications $USER_DIR/*

#Add two items to the CKill whitelist
if [ "x`grep su /etc/ckill/whitelist`" == "x" ] ; then
    echo "su" >> /etc/ckill/whitelist
fi                                                                                
if [ "x`grep yes /etc/ckill/whitelist`" == "x" ]; then
    echo "yes" >> /etc/ckill/whitelist
fi

echo "Copy scripts"
PR2_APP_MAN_PKG=`su applications -c "source ~/ros/setup.bash && rospack find pr2_app_manager 2> /dev/null"`
APP_MAN_PKG=`su applications -c "source ~/ros/setup.bash && rospack find app_manager 2> /dev/null"`
WILLOW_MAPS_PKG=`su applications -c "source ~/ros/setup.bash && rospack find willow_maps 2> /dev/null"`

echo "Add control.py"
cp $PR2_APP_MAN_PKG/scripts/control.py /usr/lib/cgi-bin
chown www-data /usr/lib/cgi-bin/control.py
chmod a+x /usr/lib/cgi-bin/control.py

echo "Add rosget"
cp $APP_MAN_PKG/scripts/rosget /usr/bin/rosget
chown root:root /usr/bin/rosget
chmod a-wrx /usr/bin/rosget
chmod ug+wrx /usr/bin/rosget

echo "Add local_apps"
mkdir -p $USER_DIR/local_apps
cat > $USER_DIR/local_apps/README <<EOF
This is a directory where you can add applications in list files so that the system can read them. See the wiki for more information.
EOF
chown -R applications $USER_DIR/local_apps

echo "ROSMake the required applications. This should not do anything"
su applications -c "source ~/ros/setup.bash && rosmake --rosdep-install pr2_app_manager app_manager"

apt-get update
if [ "x`apt-cache dump | grep ros-electric-map-store`" == "x" ] ; then
    echo "No map store available"
else
    echo "Map store exists"
    apt-get install ros-electric-map-store -y
    echo "Install the willowgarage map into the map_store"
    su applications -c "source ~/ros/setup.bash && roslaunch map_store add_map.launch \"map_file:=$WILLOW_MAPS_PKG/willow-sans-whitelab-2010-02-18-0.025.pgm\" \"map_package:=willow_maps\" \"map_name:=willow_garage\" \"map_resolution:=0.025\""
fi

echo "Check for sudo rule"
if [ "x`grep rosget /etc/sudoers`" == "x" ] ; then
    echo "Append to sudo"
    cat > /root/append_to_sudo.sh <<EOF
#!/bin/bash
echo "" >> \$1
echo "%rosget   ALL=(ALL) NOPASSWD: /usr/bin/rosget" >> \$1\\
echo "" >> \$1
EOF
    chmod a+x /root/append_to_sudo.sh
    EDITOR="/root/append_to_sudo.sh" visudo
    rm /root/append_to_sudo.sh
else
    echo "Sudo rule already present"
fi





