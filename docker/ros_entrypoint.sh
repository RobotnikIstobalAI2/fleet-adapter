#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"
source "/home/ros/ros2_ws/install/setup.bash"
#source "/opt/ros/noetic/setup.bash"
#source "/home/ros/catkin_ws/devel/setup.bash"
exec "$@"
