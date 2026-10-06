ARG YOLO_BASE_IMAGE=hydronautics/od:l4t-r36.3.0
FROM ${YOLO_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=1
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      ros-humble-cv-bridge \
      ros-humble-image-transport && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /welt_auv
COPY . /welt_auv

RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-select \
        stingray_interfaces \
        stingray_object_detection \
        sauvc_object_detection \
        stingray_launch \
        welt_launch

COPY docker/od/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /models

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "welt_launch", "yolo.launch.py", "weights_path:=/models/yolov8.pt"]
