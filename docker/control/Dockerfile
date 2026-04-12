FROM hydronautics/stingray:control

WORKDIR /welt_auv
# COPY . /welt_auv
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && source /stingray_core/install/setup.bash && source /stingray/install/setup.bash && source /sauvc/install/setup.bash && colcon build --packages-select  welt_launch"

COPY docker/control/entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

RUN echo 'source /welt_auv/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
