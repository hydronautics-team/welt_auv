#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common.sh"

usage() {
  cat <<USAGE
Использование:
  $0 <target> [--force]

Где <target>:
  build    - удалить build/ install/ log/
  records  - удалить records/
USAGE
}

[[ $# -ge 1 && $# -le 2 ]] || {
  usage
  exit 1
}

target="$1"
force="false"

if [[ $# -eq 2 ]]; then
  [[ "$2" == "--force" ]] || die "Неизвестный флаг: $2"
  force="true"
fi

ROOT_DIR="$(repo_root_from_script "${BASH_SOURCE[0]}")"
cd "$ROOT_DIR"

paths=()
case "$target" in
  build)
    paths=(build install log)
    ;;
  records)
    paths=(records)
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    die "Неизвестная цель очистки: $target"
    ;;
esac

if [[ "$force" != "true" ]]; then
  log_warn "Будут удалены: ${paths[*]}"
  read -r -p "Подтвердите удаление (yes/no): " answer
  [[ "$answer" == "yes" ]] || die "Отменено пользователем"
fi

for p in "${paths[@]}"; do
  if [[ -e "$ROOT_DIR/$p" ]]; then
    rm -rf -- "$ROOT_DIR/$p"
    log_info "Удалено: $p"
  else
    log_warn "Пропущено (не найдено): $p"
  fi
done
