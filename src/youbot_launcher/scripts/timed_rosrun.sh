#!/bin/bash
#
# Script to delay the launch of a rosrun command
#
# Use: ./timed_rosrun.sh [number of seconds to delay] [rospkg] [node name]
#

echo "Waiting $1 seconds for 'rosrun $@'"
sleep $1
shift
rosrun $@
