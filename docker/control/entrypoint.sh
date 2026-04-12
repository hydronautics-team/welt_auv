#!/bin/bash
set -e

# echo "[DOCKER-INFO] Запускаем сборку..."
# Выполняем сборку, при ошибке выходим

source "/opt/ros/humble/setup.bash"
source "/additional_packages/install/setup.bash"
if [ -f /welt_auv/install/setup.bash ]; then
  source /welt_auv/install/setup.bash
else
  echo "[DOCKER-WARN] /welt_auv/install/setup.bash не найден."
  echo "[DOCKER-WARN] Похоже, workspace смонтирован с хоста без предварительной сборки."
  echo "[DOCKER-WARN] Выполните './bc' в корне проекта и перезапустите контейнер."
fi

# if ! ./bc; then
#   echo "[DOCKER-ERROR] Сборка завершилась с ошибкой. Выходим..."
#   exit 1
# fi
# echo "[DOCKER-INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
# source /welt_auv/install/setup.bash

exec "$@"
