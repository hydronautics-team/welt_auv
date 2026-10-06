#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

if [[ -z "${YOLO_WEIGHTS:-}" || ! -f "$YOLO_WEIGHTS" ]]; then
  echo "Set YOLO_WEIGHTS to an existing weights file in docker/.env or the environment." >&2
  exit 1
fi

docker compose up -d --build yolo
