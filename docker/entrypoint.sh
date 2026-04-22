#!/usr/bin/env bash
# Source base Humble (/opt/ros/humble built from source by the dustynv image)
# then our rosbridge overlay at /ws/install, then hand off to the compose
# command.
set -e
source /opt/ros/humble/install/setup.bash
source /ws/install/setup.bash
exec "$@"
