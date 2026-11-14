FROM hydronautics/stingray:control

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV ROS_DOMAIN_ID=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl && \
    curl -fsSL \
      https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb \
      -o /tmp/ros2-apt-source.deb && \
    rm -f /etc/apt/sources.list.d/ros2.list \
      /etc/apt/sources.list.d/ros2-latest.list && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp && \
    rm -f /tmp/ros2-apt-source.deb && rm -rf /var/lib/apt/lists/*

WORKDIR /welt_auv
COPY . /welt_auv

RUN source /opt/ros/humble/setup.bash && \
    if [[ -f /additional_packages/install/setup.bash ]]; then source /additional_packages/install/setup.bash; fi && \
    colcon build \
      --packages-select \
        stingray_interfaces \
        stingray_utils \
        stingray_planning \
        stingray_missions \
        stingray_object_detection \
        sauvc_interfaces \
        sauvc_missions \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/control/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "stingray_planning", "mission_pool_test.launch.py"]
