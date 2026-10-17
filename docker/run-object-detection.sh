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

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This stack requires an ARM64 Jetson host; detected $(uname -m)." >&2
  exit 1
fi

docker compose up -d --build zed yolo
docker compose ps zed yolo

echo
echo "Detection output: ${ZED_IMAGE_TOPIC:-/zed/zed_node/rgb/color/rect/image}/bbox_array"
echo "Follow logs with: docker compose -f docker/docker-compose.yml logs -f zed yolo"
