#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
for setup_file in \
  /additional_packages/install/setup.bash \
  /welt_auv/install/setup.bash
do
  if [[ -f "${setup_file}" ]]; then
    source "${setup_file}"
  fi
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
exec "$@"
