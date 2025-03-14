#!/bin/bash
set -e

# echo "[DOCKER-INFO] Запускаем сборку..."
# Выполняем сборку, при ошибке выходим

source "/opt/ros/humble/setup.bash"
source "/additional_packages/install/setup.bash"
source /welt_auv/install/setup.bash

# if ! ./bc; then
#   echo "[DOCKER-ERROR] Сборка завершилась с ошибкой. Выходим..."
#   exit 1
# fi
# echo "[DOCKER-INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
# source /welt_auv/install/setup.bash

exec "$@"
