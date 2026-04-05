#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/lib/common.sh"

usage() {
  cat <<USAGE
Использование:
  $0 <profile>

Профили:
  control   - ros2 launch welt_launch control.launch.py
  od        - ros2 launch welt_launch od.launch.py
  sim       - ros2 launch welt_launch sim.launch.py
USAGE
}

[[ $# -eq 1 ]] || {
  usage
  exit 1
}

profile="$1"

[[ -f install/setup.bash ]] || die "Не найден install/setup.bash. Сначала соберите workspace."

# shellcheck source=/dev/null
set +u
source install/setup.bash
set -u

case "$profile" in
  control)
    exec ros2 launch welt_launch control.launch.py
    ;;
  od)
    exec ros2 launch welt_launch od.launch.py
    ;;
  sim)
    exec ros2 launch welt_launch sim.launch.py
    ;;
  -h|--help)
    usage
    ;;
  *)
    die "Неизвестный профиль: $profile"
    ;;
esac
