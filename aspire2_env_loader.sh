#!/usr/bin/env bash

export ROS_HOSTNAME=tp15-aspire-2
export ROS_MASTER_URI='http://y-master:11311'
export ROS_IP=tp15-aspire-2

source /opt/ros/kinetic/setup.bash
source ~/mars/devel_isolated/setup.bash

exec "$@"