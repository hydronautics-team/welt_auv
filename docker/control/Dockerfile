FROM hydronautics/stingray:control

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV ROS_DOMAIN_ID=1

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
        sauvc_interfaces \
        sauvc_missions \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/control/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "stingray_planning", "mission_pool_test.launch.py"]
