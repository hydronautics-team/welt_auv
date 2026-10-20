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

camera_type="${FRONT_CAMERA_TYPE:-zed}"
case "$camera_type" in
  zed)
    export FRONT_CAMERA_IMAGE_TOPIC="${ZED_IMAGE_TOPIC:-/zed/zed_node/rgb/color/rect/image}"
    export FRONT_CAMERA_INFO_TOPIC="${ZED_CAMERA_INFO_TOPIC:-/zed/zed_node/rgb/color/rect/camera_info}"
    docker compose --profile usb-camera stop front-camera
    docker compose up -d --build zed yolo
    docker compose ps zed yolo
    ;;
  usb)
    export FRONT_CAMERA_IMAGE_TOPIC='/stingray/topics/camera/front'
    export FRONT_CAMERA_INFO_TOPIC='/stingray/topics/camera/front/camera_info'
    if [[ ! -e "${USB_CAMERA_DEVICE:-/dev/video0}" ]]; then
      echo "USB camera is missing: ${USB_CAMERA_DEVICE:-/dev/video0}" >&2
      exit 1
    fi
    docker compose stop zed
    docker compose --profile usb-camera up -d --build front-camera yolo
    docker compose --profile usb-camera ps front-camera yolo
    ;;
  *)
    echo "FRONT_CAMERA_TYPE must be 'zed' or 'usb', got: $camera_type" >&2
    exit 2
    ;;
esac

echo
echo "Front camera: $camera_type"
echo "Image input: $FRONT_CAMERA_IMAGE_TOPIC"
echo "Detection output: $FRONT_CAMERA_IMAGE_TOPIC/bbox_array"
echo "Follow logs with: docker compose -f docker/docker-compose.yml logs -f"
