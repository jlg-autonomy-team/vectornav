#!/bin/bash

# Update and download dependencies
sudo apt-get update
local_deps.sh
rosdep update
rosdep install --from-paths ${USER_WORKSPACE}/src --ignore-src --rosdistro=${ROS_DISTRO} -y -r
