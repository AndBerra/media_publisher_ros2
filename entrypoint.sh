#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

cd /ros2_ws

echo "--- Building ROS 2 workspace from mounted volume ---"
colcon build --symlink-install

echo "--- Sourcing built workspace ---"
source /ros2_ws/install/setup.bash

exec "$@"