#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ws/install/setup.bash"

export ROS_DOMAIN_ID=30 # setup domain id
ros2 launch rae_hw control.launch.py enable_battery_status:=false # spustil launch
