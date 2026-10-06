#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source /opt/zed_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
exec "$@"
