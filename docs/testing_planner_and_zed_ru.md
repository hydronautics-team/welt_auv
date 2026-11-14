# Проверка планировщика и распознавания с ZED

Этот документ описывает две независимые проверки:

1. `welt_auv/control`: планировщик получает цель и публикует `Twist` в
   `/control/data`; `welt_rov` предоставляет локализацию, DVL и принимает
   команды управления.
2. `welt_auv/zed` + `welt_auv/yolo`: ZED публикует изображение, YOLO получает
   его и публикует `BboxArray` и, при включённом debug, изображение с рамками.

Команды ниже выполняются из корня `welt_auv`, если явно не указано иное.

## 1. Требования и подготовка

Для полного комплекта ZED + YOLO нужен Jetson ARM64 с NVIDIA Container
Toolkit, доступной камерой ZED и ресурсами ZED SDK на хосте. На обычном x86_64
компьютере `run-object-detection.sh` намеренно завершится с ошибкой.

Все ROS 2 процессы должны использовать одинаковые настройки discovery:

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Создайте локальную конфигурацию:

```bash
cp -n docker/.env.example docker/.env
```

Для ZED в `docker/.env` должны быть как минимум такие значения:

```dotenv
ROS_DOMAIN_ID=1
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FRONT_CAMERA_TYPE=zed
ZED_CAMERA_MODEL=zedm
ZED_IMAGE_TOPIC=/zed/zed_node/rgb/color/rect/image
ZED_CAMERA_INFO_TOPIC=/zed/zed_node/rgb/color/rect/camera_info
YOLO_ENABLED_ON_START=True
YOLO_DEBUG=True
YOLO_WEIGHTS=../src/sauvc/src/sauvc_object_detection/weights/yolov8.pt
```

Проверьте наличие весов:

```bash
test -f src/sauvc/src/sauvc_object_detection/weights/yolov8.pt \
  && echo "weights: OK" \
  || echo "weights: MISSING"
```

Файл весов не хранится в Git. Если он лежит в другом месте, укажите путь в
`YOLO_WEIGHTS` относительно каталога `docker` либо абсолютным путём.

Перед тестом полезно проверить Docker и GPU:

```bash
docker version
docker info | grep -i runtime
nvidia-smi
lsusb | grep -i stereolabs
```

На Jetson `nvidia-smi` может показывать меньше информации, чем на настольной
GPU. Важнее, чтобы Docker видел runtime `nvidia`, а ZED была видна через USB.

## 2. Проверка планировщика

### 2.1. Безопасность

Если `welt_rov` подключён к настоящему аппарату, публикация цели может привести
к запуску движителей. Для стендовой проверки сначала аппарат должен быть
надёжно зафиксирован, движители отключены аппаратно или запуск должен идти в
режиме, в котором силовые выходы запрещены.

Пассивные команды `ros2 node list`, `ros2 topic list`, `ros2 topic info` и
`ros2 topic echo` безопасны: они ничего не публикуют.

### 2.2. Запуск `welt_rov`

На компьютере управления запустите штатный bringup `welt_rov` по принятой в
проекте процедуре. Если workspace уже собран, базовая команда выглядит так:

```bash
cd /home/cherry/Hydronautics/welt_rov
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch welt_bringup run_rov.launch.py
```

Этот bringup должен предоставить планировщику:

- `/core/state/odometry` (`nav_msgs/msg/Odometry`);
- `/stingray_core/sensors/dvl/odometry` (`nav_msgs/msg/Odometry`);
- подписчика `/control/data` (`geometry_msgs/msg/Twist`).

Если проверяется только ROS-цепочка, используйте безопасный стендовый режим
`welt_rov`; не подменяйте аппаратную процедуру этой инструкцией.

### 2.3. Запуск планировщика

В новом терминале:

```bash
cd /home/cherry/Hydronautics/welt_auv
./run_pool_planner.sh
```

Скрипт пересобирает `control`-образ и запускает
`stingray_planning pool_test.launch.py`. В этом тестовом launch работают:

- `/path_planning`;
- `/pool_empty_map`, публикующий пустую карту 20 x 20 м в frame `odom`.

Ожидаемые строки в логах:

```text
process started ... path_planning_node
process started ... empty_map_publisher
Published 20.0 x 20.0 m empty pool map in frame 'odom'
```

Чтобы оставить контейнер запущенным и выйти из просмотра логов, нажмите
`Ctrl+C`. Контейнер при этом не останавливается.

### 2.4. Проверка ROS-графа

Откройте shell с настроенным ROS внутри control-контейнера:

```bash
docker exec -it welt_auv_control bash -lc \
  'source /opt/ros/humble/setup.bash; source /welt_auv/install/setup.bash; exec bash'
```

Внутри контейнера выполните:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info -v /map/occupancy_local
ros2 topic info -v /core/state/odometry
ros2 topic info -v /stingray_core/sensors/dvl/odometry
ros2 topic info -v /control/data
```

Перед отправкой цели ожидается:

| Топик | Минимальное состояние |
| --- | --- |
| `/map/occupancy_local` | 1 publisher (`pool_empty_map`) |
| `/core/state/odometry` | 1 publisher из `welt_rov` |
| `/stingray_core/sensors/dvl/odometry` | 1 publisher из `welt_rov` |
| `/control/data` | 1 publisher планировщика и 1 subscriber из `welt_rov` |

Проверьте частоты и сообщения:

```bash
ros2 topic hz /core/state/odometry
ros2 topic hz /stingray_core/sensors/dvl/odometry
ros2 topic echo /core/state/odometry --once
ros2 topic echo /stingray_core/sensors/dvl/odometry --once
```

У обоих сообщений должны обновляться `header.stamp`. Frame одометрии должен
совпадать с frame тестовой карты — `odom`. Планировщик намеренно отклоняет
маршрут, если frame карты, цели и одометрии не совпадают.

### 2.5. Наблюдение за выходом и отправка цели

Сначала откройте отдельный терминал и подпишитесь на команды:

```bash
docker exec -it welt_auv_control bash -lc \
  'source /opt/ros/humble/setup.bash; source /welt_auv/install/setup.bash; \
   ros2 topic echo /control/data'
```

Публиковать цель можно только после выполнения мер безопасности из раздела
2.1. Пример цели `x=2`, `y=1` в frame `odom`:

```bash
docker exec -it welt_auv_control bash -lc \
  'source /opt/ros/humble/setup.bash; source /welt_auv/install/setup.bash; \
   ros2 topic pub --once /planner/goal geometry_msgs/msg/PoseStamped \
   "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"'
```

Успешный тест выглядит так:

1. В логе появляется `Planned route with ... waypoints`.
2. В `/planner/global_path` появляется маршрут:

   ```bash
   ros2 topic echo /planner/global_path --once
   ```

3. `/control/data` публикуется примерно с частотой `control_rate_hz` (по
   умолчанию 10 Гц):

   ```bash
   ros2 topic hz /control/data
   ```

4. В командах `linear.z` равно целевой глубине `0.5`, а `linear.x`,
   `linear.y` и при необходимости `angular.z` задают движение к waypoint.
5. После достижения цели появляется `Goal reached` и публикуется команда
   удержания с нулевым горизонтальным движением.

Первое сообщение только с `linear.z: 0.5` является командой остановки и
удержания глубины при принятии новой цели. Если после него нет команд движения,
смотрите диагностику ниже.

### 2.6. Диагностика планировщика

Посмотреть последние логи:

```bash
docker compose -f docker/docker-compose.yml logs --tail=200 control
```

Типичные причины:

- `Waiting for occupancy grid before planning` — нет издателя карты;
- `Waiting for odometry before planning` — не виден `welt_rov` или не запущена
  локализация;
- `Odometry timeout; stopping` — одометрия не приходит, приходит реже таймаута
  0.5 с или имеет старый/нулевой `header.stamp`;
- `Velocity observation timeout; stopping` — нет свежей DVL odometry;
- `Goal frame ... differs from map frame ...` — цель отправлена не в `odom`;
- `Odometry frame ... differs from map frame ...` — требуется исправить frame
  локализации или добавить TF-преобразование;
- `Start or goal lies outside the occupancy grid` — координаты за пределами
  `[-10, 10)` по x или y для стандартной тестовой карты;
- `Path not found` — маршрут заблокирован картой.

Параметры активной ноды:

```bash
ros2 param dump /path_planning
```

Если `/control/data` имеет publisher, но не имеет subscriber, проверьте на
обеих сторонах одинаковые `ROS_DOMAIN_ID` и `RMW_IMPLEMENTATION`, затем
перезапустите ROS 2 daemon:

```bash
ros2 daemon stop
ros2 daemon start
```

## 3. Проверка ZED и YOLO

### 3.1. Запуск полного конвейера

Из корня `welt_auv`:

```bash
./docker/run-object-detection.sh
```

Скрипт проверит архитектуру ARM64 и наличие весов, затем пересоберёт и запустит
`zed` и `yolo`. После запуска он напечатает фактические входной и выходной
топики.

Следите за логами:

```bash
docker compose -f docker/docker-compose.yml logs -f zed yolo
```

Остановить просмотр можно через `Ctrl+C`; контейнеры продолжат работать.

Проверить состояние:

```bash
docker compose -f docker/docker-compose.yml ps zed yolo
```

Оба сервиса должны иметь состояние `Up`. Циклический restart означает ошибку
камеры, GPU, модели или launch-файла — смотрите логи соответствующего сервиса.

### 3.2. Проверка ZED отдельно

Откройте ROS-shell внутри ZED-контейнера:

```bash
docker exec -it welt_auv_zed bash -lc \
  'source /opt/ros/humble/setup.bash; source /opt/zed_ros2_ws/install/setup.bash; exec bash'
```

Проверьте ноды и топики:

```bash
ros2 node list
ros2 topic list | grep /zed
ros2 topic info -v /zed/zed_node/rgb/color/rect/image
ros2 topic info -v /zed/zed_node/rgb/color/rect/camera_info
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic hz /zed/zed_node/rgb/color/rect/camera_info
```

У image и camera_info должен быть publisher. После запуска YOLO у image также
должен появиться subscriber.

Если указанных топиков нет, найдите реальные имена:

```bash
ros2 topic list -t | grep -E 'zed|Image|CameraInfo'
```

После этого запишите их в `docker/.env` как `ZED_IMAGE_TOPIC` и
`ZED_CAMERA_INFO_TOPIC` и перезапустите конвейер.

### 3.3. Проверка детектора

Стандартные топики для ZED:

```text
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/color/rect/camera_info
/zed/zed_node/rgb/color/rect/image/bbox_array
/zed/zed_node/rgb/color/rect/image/debug_image
```

`debug_image` существует только при `YOLO_DEBUG=True`.

Откройте ROS-shell внутри YOLO-контейнера:

```bash
docker exec -it welt_auv_yolo bash -lc \
  'source /opt/ros/humble/setup.bash; source /welt_auv/install/setup.bash; exec bash'
```

Проверьте связи и частоты:

```bash
ros2 node list
ros2 topic info -v /zed/zed_node/rgb/color/rect/image
ros2 topic info -v /zed/zed_node/rgb/color/rect/image/bbox_array
ros2 topic hz /zed/zed_node/rgb/color/rect/image/bbox_array
ros2 topic echo /zed/zed_node/rgb/color/rect/image/bbox_array --once
```

`bbox_array` может быть пустым, если в кадре нет объектов известных модели;
это не ошибка. Важны регулярное появление сообщений и отсутствие исключений в
логе YOLO.

Для просмотра изображения с рамками на машине с ROS GUI:

```bash
rqt_image_view /zed/zed_node/rgb/color/rect/image/debug_image
```

Либо выберите этот топик вручную в `rqt_image_view`. Если GUI запускается в
контейнере, хост должен разрешать X11-доступ; проще запускать viewer из
совместимого ROS workspace на хосте.

### 3.4. Диагностика ZED/YOLO

**ZED не стартует:**

```bash
docker compose -f docker/docker-compose.yml logs --tail=200 zed
ls -l /dev/video* 2>/dev/null
lsusb | grep -i stereolabs
```

Проверьте `ZED_CAMERA_MODEL`, USB-подключение, питание камеры, совместимость
JetPack/L4T и ZED SDK.

**YOLO не стартует:**

```bash
docker compose -f docker/docker-compose.yml logs --tail=200 yolo
docker exec welt_auv_yolo test -r /models/yolov8.pt
```

Проверьте путь к весам, доступ GPU и совместимость базового образа с JetPack.

**ZED публикует, но YOLO не получает изображение:**

```bash
ros2 topic info -v /zed/zed_node/rgb/color/rect/image
```

Должны быть publisher от ZED и subscriber от YOLO. Если subscriber отсутствует,
сравните фактический топик с `ZED_IMAGE_TOPIC`, а также `ROS_DOMAIN_ID` и
`RMW_IMPLEMENTATION` обоих контейнеров.

**Есть bbox, но нет debug image:** включите `YOLO_DEBUG=True` в `docker/.env` и
пересоздайте YOLO:

```bash
docker compose -f docker/docker-compose.yml up -d --build --force-recreate yolo
```

## 4. Одновременная проверка всей системы

Порядок запуска:

1. Запустить безопасный bringup `welt_rov`.
2. Запустить планировщик: `./run_pool_planner.sh`.
3. Запустить ZED + YOLO: `./docker/run-object-detection.sh`.
4. Проверить ROS-граф и частоты всех обязательных топиков.
5. Сначала проверить изображение и `bbox_array` пассивно.
6. Только после проверки мер безопасности отправить `/planner/goal`.
7. Наблюдать `/planner/global_path`, `/control/data` и логи control.

Краткая итоговая проверка:

```bash
ros2 topic info -v /core/state/odometry
ros2 topic info -v /stingray_core/sensors/dvl/odometry
ros2 topic info -v /control/data
ros2 topic info -v /zed/zed_node/rgb/color/rect/image
ros2 topic info -v /zed/zed_node/rgb/color/rect/image/bbox_array
```

## 5. Остановка

Остановить только сервисы `welt_auv`:

```bash
docker compose -f docker/docker-compose.yml stop control zed yolo
```

Удалить контейнеры и сеть compose без удаления образов:

```bash
docker compose -f docker/docker-compose.yml down
```

`welt_rov` останавливается отдельно штатным способом в его терминале.
