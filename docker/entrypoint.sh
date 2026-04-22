#!/usr/bin/env bash
# Source ROS2 Humble and then hand off to whatever command compose passed.
set -e
source /opt/ros/humble/setup.bash
exec "$@"
