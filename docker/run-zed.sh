#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

docker compose up -d --build zed
exec docker compose exec zed bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/zed_ros2_ws/install/setup.bash && exec bash -i'
