#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
mode="--synthetic"
rebuild=false
services=(sim-mission)
profiles=(--profile simulator)

for argument in "$@"; do
  case "$argument" in
    --synthetic|--yolo)
      mode="$argument"
      ;;
    --build)
      rebuild=true
      ;;
    *)
      echo "Usage: $0 [--synthetic|--yolo] [--build]" >&2
      exit 2
      ;;
  esac
done

case "$mode" in
  --synthetic)
    export SIM_SYNTHETIC_DETECTIONS=true
    ;;
  --yolo)
    if ! command -v nvidia-smi >/dev/null 2>&1 || \
       ! nvidia-smi -L >/dev/null 2>&1; then
      echo "NVIDIA GPU is present but its driver is unavailable." >&2
      echo "Fix nvidia-smi and install NVIDIA Container Toolkit before using --yolo." >&2
      exit 4
    fi
    weights_path="${YOLO_WEIGHTS:-../src/sauvc/src/sauvc_object_detection/weights/yolov8.pt}"
    if [[ ! -r "$weights_path" ]]; then
      echo "YOLO weights are missing: $weights_path" >&2
      echo "Put the gate model there or set YOLO_WEIGHTS=/absolute/path/model.pt" >&2
      exit 1
    fi
    export SIM_SYNTHETIC_DETECTIONS=false
    profiles+=(--profile simulator-yolo)
    services+=(sim-yolo)
    ;;
esac

if [[ "$rebuild" == "true" ]]; then
  docker compose "${profiles[@]}" build "${services[@]}"
fi
docker compose "${profiles[@]}" up -d --force-recreate "${services[@]}"
docker compose "${profiles[@]}" ps "${services[@]}"

if [[ "${SIM_START_MISSION:-true}" == "true" ]]; then
  echo "Waiting for simulator odometry..."
  if ! docker compose "${profiles[@]}" exec -T sim-mission bash -lc '
      source /opt/ros/humble/setup.bash
      source /welt_auv/install/setup.bash
      timeout 30 ros2 topic echo /core/state/odometry nav_msgs/msg/Odometry --once >/dev/null
    '; then
    echo "No /core/state/odometry. Start ~/Hydronautics/sauvc_simulation_assets/run-simulator.sh first." >&2
    exit 3
  fi
  docker compose "${profiles[@]}" exec -T sim-mission bash -lc '
    source /opt/ros/humble/setup.bash
    source /welt_auv/install/setup.bash
    ros2 service call /stingray/services/transition \
      stingray_interfaces/srv/SetTransition \
      "{transition: gate_sauvc_2027}"
  '
fi

echo
echo "Simulator mission mode: $mode"
echo "Native Stingray topics use ROS_DOMAIN_ID=$ROS_DOMAIN_ID."
echo "Mission transition was requested automatically."
echo "Logs: docker compose -f docker/docker-compose.yml --profile simulator logs -f sim-mission"
