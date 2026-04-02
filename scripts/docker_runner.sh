#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/lib/common.sh"

require_cmd docker

usage() {
  cat <<USAGE
Использование:
  $0 <scenario>

Сценарии:
  control_shell             - интерактивный shell в control-контейнере с mount workspace
  control_launch            - запуск control.launch.py
  control_detached_launch   - detached запуск control-контейнера с командой 'bash lc'
  od_shell                  - интерактивный shell в od-контейнере с GPU и /dev
  od_launch                 - запуск od.launch.py с GPU
  od_detached_launch        - detached запуск od-контейнера с командой 'bash lv'
USAGE
}

[[ $# -eq 1 ]] || {
  usage
  exit 1
}

scenario="$1"

run_or_attach() {
  local container_name="$1"
  shift

  local running_container
  running_container=$(docker ps -q -f name="^${container_name}$")
  if [[ -n "$running_container" ]]; then
    log_info "Контейнер ${container_name} уже запущен. Подключаемся..."
    docker exec -it "$container_name" bash
    exit 0
  fi

  local stopped_container
  stopped_container=$(docker ps -aq -f name="^${container_name}$")
  if [[ -n "$stopped_container" ]]; then
    log_info "Контейнер ${container_name} найден в stopped. Запускаем..."
    docker start "$container_name" >/dev/null
    docker exec -it "$container_name" bash
    exit 0
  fi

  log_info "Контейнер ${container_name} не найден. Создаем..."
  docker run "$@"
}

case "$scenario" in
  control_shell)
    run_or_attach "welt_auv_control" -it --rm \
      --name "welt_auv_control" \
      --mount type=bind,source="$(pwd)",target=/welt_auv \
      --mount type=bind,source="/etc/timezone",target="/etc/timezone" \
      --mount type=bind,source="/etc/localtime",target="/etc/localtime" \
      --ipc=host \
      --net=host \
      hydronautics/welt_auv:control bash
    ;;

  control_launch)
    run_or_attach "welt_auv_control" -it --rm \
      --name "welt_auv_control" \
      --mount type=bind,source="/etc/timezone",target="/etc/timezone" \
      --mount type=bind,source="/etc/localtime",target="/etc/localtime" \
      --ipc=host \
      --net=host \
      hydronautics/welt_auv:control ros2 launch welt_launch control.launch.py
    ;;

  control_detached_launch)
    run_or_attach "welt_auv_control" -d \
      --name "welt_auv_control" \
      --restart unless-stopped \
      --mount type=bind,source="$(pwd)",target=/welt_auv \
      --mount type=bind,source="/etc/timezone",target="/etc/timezone" \
      --mount type=bind,source="/etc/localtime",target="/etc/localtime" \
      --ipc=host \
      --net=host \
      hydronautics/welt_auv:control bash lc
    ;;

  od_shell)
    run_or_attach "welt_auv_od" -it --rm --platform=linux/arm64 --gpus all --runtime nvidia \
      --name "welt_auv_od" \
      --mount type=bind,source="$(pwd)",target=/welt_auv \
      --mount type=bind,source="/dev",target=/dev \
      --mount type=bind,source="/etc/timezone",target="/etc/timezone" \
      --mount type=bind,source="/etc/localtime",target="/etc/localtime" \
      --device=/dev/video0 \
      --ipc=host \
      --net=host \
      hydronautics/welt_auv:od bash
    ;;

  od_launch)
    run_or_attach "welt_auv_od" -it --rm --platform=linux/arm64 --gpus all --runtime nvidia \
      --name "welt_auv_od" \
      --mount type=bind,source="/etc/timezone",target="/etc/timezone" \
      --mount type=bind,source="/etc/localtime",target="/etc/localtime" \
      --ipc=host \
      --net=host \
      hydronautics/welt_auv:od ros2 launch welt_launch od.launch.py
    ;;

  od_detached_launch)
    run_or_attach "welt_auv_od" -d --platform=linux/arm64 --gpus all --runtime nvidia \
      --name "welt_auv_od" \
      --restart unless-stopped \
      --mount type=bind,source="$(pwd)",target=/welt_auv \
      --mount type=bind,source="/dev",target=/dev \
      --mount type=bind,source="/etc/timezone",target="/etc/timezone" \
      --mount type=bind,source="/etc/localtime",target="/etc/localtime" \
      --device=/dev/video0 \
      --ipc=host \
      --net=host \
      hydronautics/welt_auv:od bash lv
    ;;

  -h|--help)
    usage
    ;;

  *)
    die "Неизвестный сценарий: $scenario"
    ;;
esac
