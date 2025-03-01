#!/bin/bash
set -e

# Проверяем, существует ли файл setup.bash
if [ -f /welt_auv/install/setup.bash ]; then
  echo "[INFO] Код уже сбилжен"
  source /welt_auv/install/setup.bash
else
  echo "[INFO] Файл install/setup.bash не найден. Запускаем сборку..."
  # Выполняем сборку, при ошибке удаляем build, install, log и выходим

  source "/opt/ros/humble/install/setup.bash"
  source "/additional_packages/install/setup.bash"

  if ! colcon build --packages-select sauvc_object_detection stingray_object_detection stingray_interfaces welt_launch sauvc_launch stingray_launch welt_cam; then
    echo "[ERROR] Сборка завершилась с ошибкой. Удаляем build, install, log..."
    rm -rf build install log
    exit 1
  fi
  echo "[INFO] Сборка завершена успешно. Выполняем source install/setup.bash..."
  source /welt_auv/install/setup.bash
fi

exec "$@"
