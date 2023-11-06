#!/bin/bash
set -e

# Check and export pwm channels if not already exported
for channel in 1 2; do
    pwm="/sys/class/pwm/pwmchip0/pwm${channel}/"
    if [ ! -d ${pwm} ]; then
        echo ${channel} > /sys/class/pwm/pwmchip0/export
    fi
    chmod -R a+rw ${pwm}
done

chmod -R a+rw /dev/gpiochip0

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ws/install/setup.bash"
source "$HOME/.bashrc"
exec "$@"
