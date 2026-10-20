ARG YOLO_BASE_IMAGE=hydronautics/od:l4t-r36.3.0
FROM ${YOLO_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=1
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

WORKDIR /welt_auv
COPY . /welt_auv

RUN source /opt/ros/humble/install/setup.bash && \
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
