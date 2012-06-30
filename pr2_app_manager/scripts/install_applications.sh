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
    yes | su applications -c "/usr/bin/check-ssh-keys"
fi

if [ "x`grep "rosget:" /etc/group`" == "x" ] ; then
    echo "Adding rosget group"
    addgroup rosget
fi

usermod -aG rosget applications

echo "Check for ROS_PACKAGE_PATH in ~/.bashrc"
if [ "x`grep ROS_PACKAGE_PATH $USER_DIR/.bashrc`" == "x" ] ; then
    echo "Add ROS_PACKAGE_PATH"
    echo "" >> $USER_DIR/.bashrc
    echo "source /etc/ros/distro/setup.bash" >> $USER_DIR/.bashrc
    echo "export ROS_ENV_LOADER=/etc/ros/distro/env.sh" >> $USER_DIR/.bashrc
    echo "export ROS_WORKSPACE=/u/pratkanis/atp_ros" >> $USER_DIR/.bashrc
    echo "export ROS_PACKAGE_PATH=$USER_DIR/apps:\$ROS_PACKAGE_PATH" >> $USER_DIR/.bashrc
else
    echo "ROS_PACKAGE_PATH already present"
fi

echo "Enter the robot name (e.g. pri):"
read ROBOT_NAME
if [ -z $ROBOT_NAME ] ; then
    echo "Must specify a robot name"
    exit 2
fi

cd $USER_DIR

echo "Add apps directory"
mkdir -p $USER_DIR/apps

echo "Set robot name"
echo "name: $ROBOT_NAME" > $USER_DIR/robot.yaml

echo "Chown applications"
chown -R applications $USER_DIR/*

#Add two items to the CKill whitelist
if [ "x`grep su /etc/ckill/whitelist`" == "x" ] ; then
    echo "su" >> /etc/ckill/whitelist
fi                                                                                
if [ "x`grep yes /etc/ckill/whitelist`" == "x" ]; then
    echo "yes" >> /etc/ckill/whitelist
fi
if [ "x`grep /bin/bash /etc/ckill/whitelist`" == "x" ]; then
    echo "/bin/bash" >> /etc/ckill/whitelist
fi

cat > $USER_DIR/sourcer.bash <<EOF
#!/bin/bash
#This is a shell script that solves the problem of the inablity of bash to source commands
. $USER_DIR/.bashrc
export PATH=$ROS_ROOT/../../bin:\$PATH
export LD_LIBRARY_PATH=$ROS_ROOT/../../lib:$LD_LIBRARY_PATH
bash -c "\$*" 2> /dev/null
EOF
chmod a+x $USER_DIR/sourcer.bash
chown -R applications $USER_DIR/*

echo "Copy scripts"
PR2_APP_MAN_PKG=`su applications -c "~applications/sourcer.bash rospack find pr2_app_manager"`
APP_MAN_PKG=`su applications -c "~applications/sourcer.bash rospack find app_manager"`
WILLOW_MAPS_PKG=`su applications -c "~applications/sourcer.bash rospack find willow_maps"`

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
su applications -c "~applications/sourcer.bash rosmake --rosdep-install pr2_app_manager app_manager"

#Get the name of the current ROS distribution, e.g. fuerte, electric, etc.
ROS_DISTRO=`echo $PR2_APP_MAN_PKG | sed 's/\// /g' | awk "{ print \\\$3; }"`

echo "type: pr2" >> $USER_DIR/robot.yaml
echo "exchange_url: https://kforge.ros.org/pr2apps/pr2_exchange/raw-file/$ROS_DISTRO" >> $USER_DIR/robot.yaml
echo "dashboard:" >> $USER_DIR/robot.yaml
echo "  class_name: ros.android.views.Pr2Dashboard" >> $USER_DIR/robot.yaml


echo "Create ROS Install"
cat > $USER_DIR/install_applications.rosinstall <<EOF
- other:
    local-name: /opt/ros/$ROS_DISTRO/ros
- other:
    local-name: /opt/ros/$ROS_DISTRO/stacks 
EOF

echo "Run rosinstall"
chown -R applications $USER_DIR/*
rm -f $USER_DIR/ros/.rosinstall
su applications -c "rosinstall $USER_DIR/ros $USER_DIR/install_applications.rosinstall"
chown -R applications $USER_DIR/*
rm $USER_DIR/install_applications.rosinstall 


echo "The current distribution is: $ROS_DISTRO"
sed -i "s/ROS_DISTRO/$ROS_DISTRO/g" /usr/lib/cgi-bin/control.py

apt-get update
if [ "x`apt-cache dump | grep ros-$ROS_DISTRO-map-store`" == "x" ] ; then
    echo "No map store available"
else
    echo "Map store exists"
    apt-get install ros-$ROS_DISTRO-map-store -y
    echo "Install the willowgarage map into the map_store"
    su applications -c "~applications/sourcer.bash roslaunch map_store add_map.launch \"map_file:=$WILLOW_MAPS_PKG/willow-sans-whitelab-2010-02-18-0.025.pgm\" \"map_package:=willow_maps\" \"map_name:=willow_garage\" \"map_resolution:=0.025\""
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




