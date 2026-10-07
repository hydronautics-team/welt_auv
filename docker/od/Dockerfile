ARG YOLO_BASE_IMAGE=hydronautics/od:l4t-r36.3.0
FROM ${YOLO_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=1
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

ARG ROS_APT_SOURCE_VERSION=1.3.0
ADD https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb /tmp/ros2-apt-source.deb

RUN rm -f /etc/apt/sources.list.d/*ros* && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      ros-humble-usb-cam && \
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
        welt_cam \
        welt_launch

COPY docker/od/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /models

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "welt_launch", "yolo.launch.py", "weights_path:=/models/yolov8.pt"]
