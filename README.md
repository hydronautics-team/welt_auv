# welt_auv
Stingray powered AUV Welt

## Контейнеры

Основные контейнеры проекта:

- `hydronautics/welt_auv:control` — контур управления AUV (`control.launch.py`)
- `hydronautics/welt_auv:od` — контур object detection (`od.launch.py`)

## Работа с Docker Compose (без дополнительных скриптов)

Используем напрямую `docker compose`:

```bash
docker compose -f docker/docker-compose.yml config
```

Профили в compose:

- `core` — `welt_auv_control`, `welt_auv_od`
- `vision` — `welt_auv_od`
- `debug` — `rviz2`, `recorder`
- `visualize` — `rviz2`

### Запуск `welt_auv_od` + `rviz2` через VNC

`rviz2` настроен на встроенный Qt VNC backend (без X11/`DISPLAY`).

```bash
cd /home/shakuevda/Desktop/hydro/ws_competition/welt_auv
cp -n docker/.env.example docker/.env

# OD + RViz2
docker compose -f docker/docker-compose.yml --profile core --profile visualize up -d welt_auv_od rviz2
```

Подключение VNC-клиентом к хосту на порт из `RVIZ2_VNC_PORT` (по умолчанию `5901`).

Проверка:

```bash
docker compose -f docker/docker-compose.yml ps
docker compose -f docker/docker-compose.yml logs -f welt_auv_od rviz2
```

Если `welt_auv_od` ругается на `/welt_auv/install/setup.bash`, пересоберите vision-образ:

```bash
./scripts/build.sh vision
docker compose -f docker/docker-compose.yml --profile core up -d --build welt_auv_od
```

Примеры:

```bash
# основной запуск
docker compose -f docker/docker-compose.yml --profile core up -d

# основной + vision
docker compose -f docker/docker-compose.yml --profile core --profile vision up -d

# debug-сервисы
docker compose -f docker/docker-compose.yml --profile debug up

# остановка
docker compose -f docker/docker-compose.yml down
```

## Переменные окружения Docker

Шаблон переменных: `docker/.env.example`

```bash
cp docker/.env.example docker/.env
```

Ключевые параметры:

- образы: `WELT_AUV_CONTROL_IMAGE`, `WELT_AUV_OD_IMAGE`
- имена контейнеров: `WELT_AUV_CONTROL_CONTAINER`, `WELT_AUV_OD_CONTAINER`
- GPU/runtime: `WELT_AUV_OD_PLATFORM`, `DOCKER_NVIDIA_RUNTIME`, `DOCKER_GPUS`
- volume-пути: `WELT_DATA_DIR`, `ROSBAG_DIR`
- запись rosbag: `RECORDER_TIMEOUT_SEC`

## Скрипты репозитория

### Сборка

Новый универсальный скрипт:

```bash
./scripts/build.sh <all|control|missions|vision>
```

Сохранённые алиасы:

- `./ba` → `build.sh all`
- `./bc` → `build.sh control`
- `./bm` → `build.sh missions`
- `./bv` → `build.sh vision`

### Docker run/attach (вне compose)

```bash
./scripts/docker_runner.sh <scenario>
```

Сценарии:

- `control_shell`
- `control_launch`
- `control_detached_launch`
- `od_shell`
- `od_launch`
- `od_detached_launch`

Алиасы:

- `dbc`, `dbcl`, `dc`, `dsim`, `dbv`, `dbvl`, `dv`

### Локальные launch-команды (без Docker)

```bash
./scripts/launch_local.sh <control|od|sim>
```

Алиасы:

- `lc`, `lv`, `lsim`

### Очистка

```bash
./scripts/cleanup.sh <build|records> [--force]
```

Алиасы:

- `delbuild [--force]`
- `delrecords [--force]`

По умолчанию очистка спрашивает подтверждение.
