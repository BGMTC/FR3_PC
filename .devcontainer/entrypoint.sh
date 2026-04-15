#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

# Only source the workspace install if it exists
# (it won't on first run before postStartCommand builds it)
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"