#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/docker"

cd "$DOCKER_DIR"

if [[ -f .env ]]; then
    set -a
    source .env
    set +a
fi

docker compose up -d --build control
exec docker compose logs -f control
