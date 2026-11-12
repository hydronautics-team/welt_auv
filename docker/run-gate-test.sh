#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

mode="${1:---dry-run}"
case "$mode" in
  --dry-run)
    export GATE_CONTROL_TOPIC=/control/data_test
    ;;
  --real)
    if [[ "${GATE_REAL_CONTROL:-}" != "YES" ]]; then
      echo "Real gate control requires GATE_REAL_CONTROL=YES." >&2
      echo "Run the dry test first, then use:" >&2
      echo "  GATE_REAL_CONTROL=YES ./docker/run-gate-test.sh --real" >&2
      exit 2
    fi
    export GATE_CONTROL_TOPIC=/control/data
    ;;
  *)
    echo "Usage: $0 [--dry-run|--real]" >&2
    exit 2
    ;;
esac

weights_path="${YOLO_WEIGHTS:-../src/sauvc/src/sauvc_object_detection/weights/yolov8.pt}"
camera_device="${USB_CAMERA_DEVICE:-/dev/video0}"
if [[ ! -f "$weights_path" ]]; then
  echo "YOLO weights are missing: $weights_path" >&2
  exit 1
fi
if [[ ! -e "$camera_device" ]]; then
  echo "USB camera is missing: $camera_device" >&2
  exit 1
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This stack requires an ARM64 Jetson host; detected $(uname -m)." >&2
  exit 1
fi

export FRONT_CAMERA_IMAGE_TOPIC=/stingray/topics/camera/front
export FRONT_CAMERA_INFO_TOPIC=/stingray/topics/camera/front/camera_info

docker compose stop control zed
docker compose --profile gate-test build gate-test
docker compose --profile usb-camera --profile gate-test up -d \
  front-camera yolo gate-test

docker compose --profile usb-camera --profile gate-test ps \
  front-camera yolo gate-test

echo
echo "Gate test mode: $mode"
echo "Control output: $GATE_CONTROL_TOPIC"
echo "Detections: /stingray/topics/camera/front/bbox_array"
echo "Start mission with transition: gate_sauvc_2027"
echo "Logs: docker compose -f docker/docker-compose.yml logs -f gate-test yolo front-camera"
