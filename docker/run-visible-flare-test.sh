#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

mode=--dry-run
build=false
for argument in "$@"; do
  case "$argument" in
    --dry-run|--real)
      mode="$argument"
      ;;
    --build)
      build=true
      ;;
    *)
      echo "Usage: $0 [--dry-run|--real] [--build]" >&2
      exit 2
      ;;
  esac
done

if [[ "$mode" == "--real" ]]; then
  if [[ "${FLARE_REAL_CONTROL:-}" != "YES" ]]; then
    echo "Real flare control requires FLARE_REAL_CONTROL=YES." >&2
    echo "Run the dry test first, then use:" >&2
    echo "  FLARE_REAL_CONTROL=YES $0 --real" >&2
    exit 2
  fi
  export FLARE_CONTROL_TOPIC=/control/data
else
  export FLARE_CONTROL_TOPIC=/control/data_test
fi

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
export YOLO_ENABLED_ON_START=True

docker compose --profile gate-test stop control zed gate-test
if [[ "$build" == true ]]; then
  docker compose --profile visible-flare-test build visible-flare-test
fi
docker compose --profile usb-camera --profile visible-flare-test up -d \
  --no-build --pull never \
  front-camera yolo visible-flare-test

docker compose --profile usb-camera --profile visible-flare-test ps \
  front-camera yolo visible-flare-test

echo
echo "Visible flare test mode: $mode"
echo "Build images: $build"
echo "Control output: $FLARE_CONTROL_TOPIC"
echo "Detections: /stingray/topics/camera/front/bbox_array"
echo "Start mission with transition: visible_flare_test"
echo "Logs: docker compose -f docker/docker-compose.yml logs -f visible-flare-test yolo front-camera"
