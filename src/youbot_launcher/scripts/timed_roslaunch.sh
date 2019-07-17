#!/bin/bash
#
# Script to delay the launch of a roslaunch command
#
# Use: ./timed_roslaunch.sh [number of seconds to delay] [rospkg] [roslaunch file]
#

echo "Waiting $1 seconds for 'roslaunch $@'"
sleep $1
shift
roslaunch $@
