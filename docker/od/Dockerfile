FROM dustynv/ros:humble-ros-base-l4t-r36.3.0@sha256:15b670796051ee3be829a559862432c5bafbdcbc93f80555dc6741a90a0f9116

RUN apt update && apt install -y --no-install-recommends git python3-pip libopenblas-dev wget

RUN wget raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh 
ENV CUDA_VERSION=12.2
RUN bash ./install_cusparselt.sh

RUN python3 -m pip install --upgrade pip; python3 -m pip install numpy==’1.26.1’; python3 -m pip install --no-cache https://developer.download.nvidia.cn/compute/redist/jp/v60/pytorch/torch-2.4.0a0+07cecf4168.nv24.05.14710581-cp310-cp310-linux_aarch64.whl

RUN pip3 install tqdm
RUN pip3 install pandas
RUN pip3 install scipy
RUN pip3 install pillow
RUN pip3 install ipython
RUN pip3 install seaborn
RUN pip3 install matplotlib

# WORKDIR /welt_auv
# COPY . /welt_auv
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && source /stingray_core/install/setup.bash && source /stingray/install/setup.bash && source /sauvc/install/setup.bash && colcon build --packages-select welt_communication welt_launch"

# COPY docker/control/entrypoint.sh /entrypoint.sh
# RUN chmod +x  /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]

RUN echo 'source /welt_auv/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
