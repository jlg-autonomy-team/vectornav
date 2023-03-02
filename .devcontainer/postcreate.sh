# Update container
sudo apt-get update

# Copy .vscode folder to workspace
ABS_SCRIPT_PATH="$(realpath "${BASH_SOURCE:-$0}")"
ABS_DIRECTORY="$(dirname "${ABS_SCRIPT_PATH}")"
ln -s ${ABS_DIRECTORY}/../.vscode $PWD

# Download dependencies
local_deps.sh
rosdep install --from-paths ${ROBOT_WS}/src --ignore-src --rosdistro=${ROS_DISTRO} -y -r
