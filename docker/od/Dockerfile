FROM hydronautics/od:l4t-r36.3.0

# WORKDIR /welt_auv
# COPY . /welt_auv
# RUN /bin/bash -c "source /opt/ros/humble/install/setup.bash && source /additional_packages/install/setup.bash && colcon build --packages-select sauvc_object_detection stingray_object_detection stingray_interfaces welt_launch sauvc_launch stingray_launch welt_cam"

COPY docker/od/entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

RUN echo 'source /welt_auv/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
