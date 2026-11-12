# Onboard containers

The onboard deployment consists of three host-networked ROS 2 containers:

- `control`: planning, mission FSM, and their Stingray interfaces/utilities;
- `zed`: ZED M ROS wrapper with camera and GPU device access;
- `yolo`: YOLO detector with NVIDIA GPU access. It consumes the ZED ROS topics.

All three containers must use the same `ROS_DOMAIN_ID`. The default is `1`, matching
`stingray_core`.

## Prepare

Put the trained model at
`src/sauvc/src/sauvc_object_detection/weights/yolov8.pt`. Alternatively, copy
the example environment file and set `YOLO_WEIGHTS` to another path:

```bash
cd docker
cp .env.example .env
$EDITOR .env
```

Select the front camera in `docker/.env`:

```dotenv
# ZED camera
FRONT_CAMERA_TYPE=zed

# Or a regular V4L2/USB camera
FRONT_CAMERA_TYPE=usb
USB_CAMERA_DEVICE=/dev/video0
```

`run-object-detection.sh` starts the selected camera, stops the other camera
service, and configures YOLO with the matching image and camera-info topics.

The repository does not contain `yolov8.pt`. Model files are excluded from Git
and from the Docker build context; the selected file is mounted read-only at
runtime.
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

For an object-detection-only test, start ZED and YOLO together from the
repository root:

```bash
./docker/run-object-detection.sh
docker compose -f docker/docker-compose.yml logs -f zed yolo
```

The run script reuses existing images and does not rebuild them. Rebuild only
after changing a Dockerfile, build arguments, dependencies, or source code:

```bash
docker compose -f docker/docker-compose.yml build zed yolo
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

`run-zed.sh` starts the service from the existing image without rebuilding it.
Build a changed ZED image explicitly with `docker compose build zed`.

`run-yolo.sh` checks that `YOLO_WEIGHTS` points to an existing weights file.
The variable can be set in `docker/.env` or exported in the shell.
After starting its service, each script opens an interactive shell in the
container with the ROS workspace `install/setup.bash` already sourced. Exit
the shell with `exit`; the container continues running in the background.

The control container starts `stingray_planning mission_pool_test.launch.py`,
including the planner, mission FSM, and empty pool map used by the integrated
go-to-point test. With ZED
Wrapper 5.1 or newer, the ZED container publishes
`/zed/zed_node/rgb/color/rect/image` and the matching `camera_info` topic. The
YOLO container consumes those topics and publishes detections at
`/zed/zed_node/rgb/color/rect/image/bbox_array`. Set `ZED_IMAGE_TOPIC` and
`ZED_CAMERA_INFO_TOPIC` in `.env` when using an older wrapper or a custom camera
namespace. Detection is enabled at startup by default; set
`YOLO_ENABLED_ON_START=False` to retain topic-controlled activation.

With `FRONT_CAMERA_TYPE=usb`, the regular camera publishes to
`/stingray/topics/camera/front`, camera calibration is published to
`/stingray/topics/camera/front/camera_info`, and detections are published to
`/stingray/topics/camera/front/bbox_array`.

## USB gate mission test

The gate test uses the new `welt_rov` automatic-control contract: a
`geometry_msgs/Twist` on `/control/data` contains bounded body-frame position
offsets, not legacy thruster percentages. The old `SetTwist` service is not
used by this mission.

Start the safe dry run first. It publishes commands to `/control/data_test`,
which is not consumed by `stingray_core`:

```bash
./docker/run-gate-test.sh --dry-run
```

After verifying camera detections, command signs, control mode, loop flags,
and the physical emergency stop, explicitly enable the real output:

```bash
GATE_REAL_CONTROL=YES ./docker/run-gate-test.sh --real
```

Start the FSM scenario with:

```bash
ros2 service call /stingray/services/transition \
  stingray_interfaces/srv/SetTransition \
  "{transition: gate_sauvc_2027}"
```

The mission first observes the initial camera sector and then searches with an
expanding yaw sweep: `+30`, `-30`, `+60`, `-60`, up to 120 degrees from the
initial heading. Three consecutive `gate` detections are required. The found
absolute yaw is preserved during centering and passage.

Visual corrections are limited to 0.75 m forward and 0.3 m laterally per
command. The final pass command is a 2.5 m relative offset. The mission stops
on stale detections or timeout and publishes a hold setpoint when interrupted.

Before a real run, start `welt_rov`/`stingray_core`, verify fresh odometry and
put control into automatic mode with all six hold loops enabled:

```bash
ros2 topic hz /core/state/odometry
ros2 topic pub --once /control/mode std_msgs/msg/UInt8 "{data: 2}"
ros2 topic pub --once /control/loop_flags std_msgs/msg/UInt8 "{data: 63}"
```

The gate does not need to be visible at startup. First verify in dry-run that
the yaw sweep follows the intended direction and that a gate on the image
right produces positive `linear.y` on `/control/data_test`; the project body
convention uses +Y right.

Emergency stop:

```bash
ros2 topic pub --once /control/data geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /control/loop_flags std_msgs/msg/UInt8 "{data: 0}"
ros2 topic pub --once /control/mode std_msgs/msg/UInt8 "{data: 0}"
docker compose -f docker/docker-compose.yml --profile gate-test stop gate-test
```

## Development mode

Container image layers are immutable. For editing, the development override
mounts this repository at `/workspace` and keeps the containers running:

```bash
./docker/dev-control.sh
docker compose exec yolo bash
```

The control helper starts the development container, opens the shell in
`/workspace`, sources ROS and the workspace overlay, and uses
`ROS_DOMAIN_ID=1` by default. The pool planner launch also publishes the static
`map -> odom` transform. Set `publish_map_odom_tf:=false` only when another
localization component already publishes that transform.

After editing on the host, rebuild the required overlay inside the container.
For control/planning and missions:

```bash
colcon build --symlink-install --packages-select \
  stingray_interfaces stingray_utils stingray_planning stingray_missions
source install/setup.bash
ros2 launch stingray_planning planning.launch.py
```

For YOLO:

```bash
cd /workspace
source /opt/ros/humble/install/setup.bash
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
