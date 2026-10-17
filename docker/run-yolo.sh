#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

weights_path="${YOLO_WEIGHTS:-../src/sauvc/src/sauvc_object_detection/weights/yolov8.pt}"
if [[ ! -f "$weights_path" ]]; then
  echo "YOLO weights are missing: $weights_path" >&2
  echo "Put the model there as yolov8.pt or set YOLO_WEIGHTS in docker/.env." >&2
  exit 1
fi
export YOLO_WEIGHTS="$weights_path"

docker compose up -d --build yolo
exec docker compose exec yolo bash -lc \
  'source /opt/ros/humble/setup.bash && source /welt_auv/install/setup.bash && exec bash -i'
