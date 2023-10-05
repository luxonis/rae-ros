#!/bin/bash
set -e

# setup ros environment
echo 2 > /sys/class/pwm/pwmchip0/export && echo 1 > /sys/class/pwm/pwmchip0/export && chmod -R a+rw /sys/class/pwm/pwmchip0/pwm1/ && chmod -R a+rw /sys/class/pwm/pwmchip0/pwm2/ && chmod -R a+rw /dev/gpiochip0
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ws/install/setup.bash"
source "$HOME/.bashrc"
exec "$@"