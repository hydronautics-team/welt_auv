#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f /welt_auv/install/setup.bash ]; then
  echo "[INFO] Код уже сбилжен"
  source /welt_auv/install/setup.bash
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  # Выполняем сборку, при ошибке удаляем build, install, log и выходим

  source "/opt/ros/humble/setup.bash"
  source "/additional_packages/install/setup.bash"
  source /stingray_core/install/setup.bash
  source /stingray/install/setup.bash
  source /sauvc/install/setup.bash

  if ! colcon build --packages-select welt_communication welt_launch; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source /welt_auv/install/setup.bash
fi

exec "$@"
