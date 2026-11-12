#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

compose_files=(-f docker-compose.yml -f docker-compose.dev.yml)

docker compose "${compose_files[@]}" up -d control
exec docker compose "${compose_files[@]}" exec \
  -w /workspace \
  control \
  bash -lc '
    source /opt/ros/humble/setup.bash
    if [[ -f /workspace/install/setup.bash ]]; then
      source /workspace/install/setup.bash
    elif [[ -f /welt_auv/install/setup.bash ]]; then
      source /welt_auv/install/setup.bash
    fi
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
    exec bash -i
  '
