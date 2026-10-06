# Onboard containers

The onboard deployment consists of three host-networked ROS 2 containers:

- `control`: only `stingray_planning`;
- `zed`: ZED M ROS wrapper with camera and GPU device access;
- `yolo`: YOLO detector with NVIDIA GPU access. It consumes the ZED ROS topics.

All three containers must use the same `ROS_DOMAIN_ID`. The default is `1`, matching
`stingray_core`.

## Prepare

Copy the example environment file and set the absolute path to the trained
weights file:

```bash
cd docker
cp .env.example .env
$EDITOR .env
```

The repository does not contain `yolov8.pt`, so `YOLO_WEIGHTS` is required.
The target computer must have NVIDIA Container Toolkit and the ZED SDK host
resources used by the official `stereolabs/zed` image.
The control and YOLO base images are stored in the Hydronautics registry; run
`docker login` before building. The ZED SDK base is the official public image.
The default ZED image is SDK 5.5 for L4T r36.5 / JetPack 6.2.2. The available
Hydronautics YOLO base is older (`r36.3.0`) but can use the newer Jetson host
driver; replace `YOLO_BASE_IMAGE` when an r36.5 build is published.
`stingray_core` runs separately and is not copied or built by these images.
The planning node communicates with it only through ROS 2 topics and publishes
commands to `/control/data`.

## Build and run

```bash
docker compose build
docker compose up -d
docker compose logs -f
```

Stop the stack with `docker compose down`. To publish the images use:

```bash
docker push hydronautics/welt_auv:control
docker push hydronautics/welt_auv:yolo
docker push hydronautics/welt_auv:zed
```

Each service can also be built and started separately:

```bash
./docker/run-control.sh
./docker/run-yolo.sh
./docker/run-zed.sh
```

`run-yolo.sh` checks that `YOLO_WEIGHTS` points to an existing weights file.
The variable can be set in `docker/.env` or exported in the shell.
After starting its service, each script opens an interactive shell in the
container with the ROS workspace `install/setup.bash` already sourced. Exit
the shell with `exit`; the container continues running in the background.

The control container starts `stingray_planning planning.launch.py`. The ZED container
publishes `/zed/zed_node/rgb/image_rect_color` and camera info. The YOLO
container consumes those topics and publishes detections under the image topic
as `bbox_array`.

## Development mode

Container image layers are immutable. For editing, the development override
mounts this repository at `/workspace` and keeps the containers running:

```bash
docker compose -f docker-compose.yml -f docker-compose.dev.yml up -d
docker compose exec control bash
docker compose exec yolo bash
```

After editing on the host, rebuild the required overlay inside the container.
For control/planning:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  stingray_planning
source install/setup.bash
ros2 launch stingray_planning planning.launch.py
```

For YOLO:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  stingray_interfaces stingray_object_detection sauvc_object_detection \
  stingray_launch welt_launch
source install/setup.bash
ros2 launch welt_launch yolo.launch.py weights_path:=/models/yolov8.pt
```

Python changes become visible through the symlink install. C++ changes, such as
`stingray_planning`, require rerunning `colcon build`. For production, rebuild
the image with `docker compose build control`, `yolo`, or `zed`; do not edit a
running production container because those changes disappear when it is
recreated.
