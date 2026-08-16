#!/bin/bash
# Handles X11 / Gazebo / ROS 2 Setup
set -e

# X11 Forwarding
if [ -z "$DISPLAY" ]; then export DISPLAY=:0; fi
if [ ! -f /tmp/.X11-unix/X0 ]; then
    Xvfb :0 -screen 0 1920x1080x24 &
    sleep 1
fi

# ROS 2
source /opt/ros/humble/setup.bash
if [ -f /opt/pacbot/ros2_ws/install/setup.bash ]; then
    source /opt/pacbot/ros2_ws/install/setup.bash
fi

exec "$@"
