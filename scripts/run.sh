#!/usr/bin/env bash
set -eo pipefail
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ros_workspace="${ROS2_WS:-$HOME/ros2_ws}"
[[ -f "$ros_setup" && -f "$ros_workspace/install/setup.bash" ]] || {
    echo "Build the demo first; check ROS_SETUP and ROS2_WS." >&2; exit 1;
}
source "$ros_setup"
source "$ros_workspace/install/setup.bash"
exec ros2 launch "$@"
