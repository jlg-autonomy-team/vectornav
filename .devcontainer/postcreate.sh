#!/bin/bash

# Update and download dependencies
sudo apt-get update
local_deps.sh
rosdep install --from-paths ${ROBOT_WS}/src --ignore-src --rosdistro=${ROS_DISTRO} -y -r
