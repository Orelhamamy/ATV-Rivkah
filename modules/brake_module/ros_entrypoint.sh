#!/bin/bash
set -e

# setup ros2 environment
source /root/ros2_ws/install/setup.bash
source /opt/ros/$ROS_DISTRO/setup.bash
exec "$@"
