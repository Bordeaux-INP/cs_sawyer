#!/bin/bash
# This is intended to be run only by System-D
# For a manual start, kill service cs_sawyer, run ../start.bash and execute the desired roslaunch

#master_hostname=`hostname`
master_hostname="021607CP00116"
hostname=`hostname`

export ROS_MASTER_URI="http://${master_hostname}.local:11311"
export ROS_HOSTNAME="${hostname}.local"

if [ -f ~/ros_ws/devel_isolated/setup.bash ]; then
 source ~/ros_ws/devel_isolated/setup.bash
fi

if [ -f ~/ros_ws/devel/setup.bash ]; then
 source ~/ros_ws/devel/setup.bash
fi

roslaunch cs_sawyer "${hostname}.launch"

