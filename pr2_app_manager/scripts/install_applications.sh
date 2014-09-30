#!/bin/bash

if [ "x`whoami`" != "xroot" ] ; then
    echo "Need to run as root. Use sudo from the pr2admin user."
    exit 1
fi

echo "Enter the robot name (e.g. pri):"
read ROBOT_NAME
if [ -z $ROBOT_NAME ] ; then
    echo "ERROR: You must specify a robot name"
    exit 2
fi

echo "Enter the ROS distribution to use"
read ROS_DISTRO
ROS_DISTRO=`echo $ROS_DISTRO | tr A-Z a-z`
if [ -z $ROS_DISTRO ] ; then
    echo "ERROR: You must specify a ROS distribution"
    exit 2
fi

if [ ! -d /etc/ros/$ROS_DISTRO ]
then
    echo "ERROR: $ROS_DISTRO does not appear to be installed"
    exit 2
fi


# create applications user
if id applications >/dev/null 2>&1
then
    echo "applications user already exists"
else
    echo "Adding user"
    echo -e "willow\nwillow\n" | adduser applications
    yes | su applications -c "/usr/bin/check-ssh-keys"
fi
USER_DIR=`getent passwd applications | cut -d: -f6`

# create rosget group
if ! getent group rosget >/dev/null 2>&1
then
    echo "Adding rosget group"
    addgroup rosget
fi

usermod -aG rosget applications

# set up .bashrc for applications user
echo "Checking for ROS_PACKAGE_PATH in ~/.bashrc"
if [ "x`grep ROS_PACKAGE_PATH $USER_DIR/.bashrc`" == "x" ] ; then
    echo "Adding ROS_PACKAGE_PATH"
    # Remove existing setup.sh/bash calls
    perl -p -i -e "s|.*/etc/ros/setup.*||" $USER_DIR/.bashrc
    # Add distro-specific setup.bash call and environment setup above non-interactive mark
    perl -n -i -e "m/If not running interactively/ and print \". /etc/ros/$ROS_DISTRO/setup.bash\nexport ROS_ENV_LOADER=/etc/ros/$ROS_DISTRO/env.sh\nexport ROS_PACKAGE_PATH=\\\$HOME/apps:\\\$ROS_PACKAGE_PATH\n\" ; print" $USER_DIR/.bashrc
else
    echo "ROS_PACKAGE_PATH is already present"
fi


cd $USER_DIR

echo "Add apps directory"
mkdir -p $USER_DIR/apps

echo "Set robot name"
echo "name: $ROBOT_NAME" > $USER_DIR/robot.yaml

echo "Chown applications"
chown -R applications $USER_DIR/*

#Add two items to the CKill whitelist
[ "x`grep su /etc/ckill/whitelist`" == "x" ] || echo "su" >> /etc/ckill/whitelist
[ "x`grep yes /etc/ckill/whitelist`" == "x" ] || echo "yes" >> /etc/ckill/whitelist
[ "x`grep /bin/bash /etc/ckill/whitelist`" == "x" ] || echo "/bin/bash" >> /etc/ckill/whitelist

chown -R applications $USER_DIR/*

PR2_APP_MAN_PKG=`su applications -c ". ~applications/.bashrc && rospack find pr2_app_manager"`
if [ -z $PR2_APP_MAN_PKG ] 
then
    echo "Unable to find package pr2_app_manager"
    exit 1
fi

APP_MAN_PKG=`su applications -c ". ~applications/.bashrc && rospack find app_manager"`
if [ -z $APP_MAN_PKG ] 
then
    echo "Unable to find package app_manager"
    exit 1
fi

WILLOW_MAPS_PKG=`su applications -c ". ~applications/.bashrc && rospack find willow_maps"`
if [ -z $WILLOW_MAPS_PKG ] 
then
    echo "Unable to find package willow_maps"
    exit 1
fi

echo "Add control.py"
cp $PR2_APP_MAN_PKG/scripts/control.py /usr/lib/cgi-bin
sed -i "s/ROS_DISTRO/$ROS_DISTRO/g" /usr/lib/cgi-bin/control.py
chown www-data /usr/lib/cgi-bin/control.py
chmod 555 /usr/lib/cgi-bin/control.py


echo "Add rosget"
cp $APP_MAN_PKG/scripts/rosget /usr/bin/rosget
chown root:root /usr/bin/rosget
chmod 550 /usr/bin/rosget

echo "Add local_apps"
mkdir -p $USER_DIR/local_apps
cat > $USER_DIR/local_apps/README <<EOF
This is a directory where you can add applications in list files so that the system can read them. See the wiki for more information.
EOF
chown -R applications $USER_DIR/local_apps

#Get the name of the current ROS distribution, e.g. fuerte, electric, etc.
#ROS_DISTRO=`echo $PR2_APP_MAN_PKG | sed 's/\// /g' | awk "{ print \\\$3; }"`

echo "type: pr2" >> $USER_DIR/robot.yaml
echo "exchange_url: https://kforge.ros.org/pr2apps/pr2_exchange/raw-file/$ROS_DISTRO" >> $USER_DIR/robot.yaml
echo "dashboard:" >> $USER_DIR/robot.yaml
echo "  class_name: ros.android.views.Pr2Dashboard" >> $USER_DIR/robot.yaml


#echo "Create ROS Install"
#cat > $USER_DIR/install_applications.rosinstall <<EOF
#- other:
#    local-name: /opt/ros/$ROS_DISTRO/ros
#- other:
#    local-name: /opt/ros/$ROS_DISTRO/stacks 
#EOF
#
#echo "Run rosinstall"
#chown -R applications $USER_DIR/*
#rm -f $USER_DIR/ros/.rosinstall
#su applications -c "rosinstall $USER_DIR/ros $USER_DIR/install_applications.rosinstall"
#chown -R applications $USER_DIR/*
#rm $USER_DIR/install_applications.rosinstall 


apt-get update
MAP_STORE_PKG="ros-$ROS_DISTRO-map-store"
echo "Searching for map_store package $MAP_STORE_PKG"
if ! apt-cache search $MAP_STORE_PKG | grep $MAP_STORE_PKG >/dev/null 2>&1
then
    echo "No map store available"
else
    echo "Map store exists"
    apt-get install ros-$ROS_DISTRO-map-store -y
    echo "Install the willowgarage map into the map_store"
    su applications -c ". ~applications/.bashrc && roslaunch map_store add_map.launch \"map_file:=$WILLOW_MAPS_PKG/willow-sans-whitelab-2010-02-18-0.025.pgm\" \"map_package:=willow_maps\" \"map_name:=willow_garage\" \"map_resolution:=0.025\""
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
