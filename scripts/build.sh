#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/lib/common.sh"

require_cmd colcon

usage() {
  cat <<USAGE
Использование:
  $0 <profile>

Профили:
  all       - полный набор (control + vision + missions)
  control   - пакеты control-контура
  missions  - только пакеты миссий
  vision    - пакеты object detection/камер
USAGE
}

[[ $# -eq 1 ]] || {
  usage
  exit 1
}

profile="$1"
packages=()

case "$profile" in
  all)
    packages=(
      welt_launch welt_cam
      sauvc_object_detection sauvc_launch sauvc_missions sauvc_movement sauvc_interfaces
      stingray_cam stingray_recorder stingray_devices stingray_object_detection stingray_interfaces
      stingray_launch stingray_missions stingray_movement stingray_utils
      stingray_core_communication stingray_core_interfaces stingray_core_launch
    )
    ;;
  control)
    packages=(
      welt_communication welt_launch
      sauvc_launch sauvc_missions sauvc_movement sauvc_interfaces
      stingray_devices stingray_interfaces stingray_launch stingray_missions stingray_movement stingray_utils
      stingray_core_communication stingray_core_interfaces stingray_core_launch
    )
    ;;
  missions)
    packages=(sauvc_missions stingray_missions)
    ;;
  vision)
    packages=(
      welt_launch welt_cam
      sauvc_object_detection sauvc_launch sauvc_interfaces
      stingray_cam stingray_recorder stingray_object_detection stingray_interfaces stingray_launch
    )
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    die "Неизвестный профиль: $profile"
    ;;
esac

log_info "Сборка профиля '$profile'"
colcon build --packages-select "${packages[@]}"
