#!/bin/bash
set -e

# Source ROS 2 base setup
source /opt/ros/humble/setup.bash

# If the workspace has already been built, source the install overlay too
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
