#!/usr/bin/env bash
set -eo pipefail

if [[ -f /opt/ros/humble/install/setup.bash ]]; then
  source /opt/ros/humble/install/setup.bash
else
  source /opt/ros/humble/setup.bash
fi
if [[ -f /welt_auv/install/setup.bash ]]; then
  source /welt_auv/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

if [[ ! -r "${YOLO_WEIGHTS_PATH:-/models/yolov8.pt}" ]]; then
  echo "[ERROR] YOLO weights are missing: ${YOLO_WEIGHTS_PATH:-/models/yolov8.pt}" >&2
  echo "[ERROR] Set YOLO_WEIGHTS in .env to the host path of yolov8.pt." >&2
  exit 64
fi

exec "$@"
