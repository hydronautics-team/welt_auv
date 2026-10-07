ARG ZED_BASE_IMAGE=stereolabs/zed:5.5-devel-l4t-r36.5
FROM ${ZED_BASE_IMAGE}

ARG ROS_DISTRO=humble
ARG ZED_WRAPPER_REF=master
ARG ROS_APT_SOURCE_VERSION=1.3.0
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_DOMAIN_ID=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ADD https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb /tmp/ros2-apt-source.deb

RUN rm -f /etc/apt/sources.list.d/*ros* && \
    apt-get update && apt-get install -y --no-install-recommends \
      curl \
      git \
      gnupg2 \
      locales \
      lsb-release \
      python3-colcon-common-extensions \
      python3-rosdep \
      software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    rm -f /etc/apt/sources.list.d/*ros* && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/zed_ros2_ws
RUN mkdir -p src && \
    git clone --depth 1 --branch "${ZED_WRAPPER_REF}" \
      https://github.com/stereolabs/zed-ros2-wrapper.git src/zed-ros2-wrapper && \
    rosdep init 2>/dev/null || true && \
    rosdep update && \
    apt-get update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/zed/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "zed_wrapper", "zed_camera.launch.py", "camera_model:=zedm"]
