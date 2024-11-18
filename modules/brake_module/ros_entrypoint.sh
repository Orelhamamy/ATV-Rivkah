#!/bin/bash
set -e

# setup ros2 environment
source /home/ATV_ws/install/setup.bash
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"
