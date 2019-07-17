#!/usr/bin/env bash

export ROS_HOSTNAME=youbot2
export ROS_MASTER_URI='http://y-master:11311'
export ROS_IP=youbot2

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/mars/devel_isolated/setup.bash

exec "$@"