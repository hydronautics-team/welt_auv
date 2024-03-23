FROM dustynv/ros:iron-pytorch-l4t-r32.7.1

RUN apt update && apt install -y --no-install-recommends git wget libboost-system-dev python3-pip software-properties-common curl graphviz graphviz-dev
RUN pip3 install pyserial transitions[diagrams]
RUN apt-get install -y libzbar-dev

RUN pip3 install tqdm
RUN pip3 install pandas
RUN apt-get install -y python3-requests build-essential libatlas-base-dev gfortran libfreetype6-dev
RUN pip3 install scipy
# RUN pip3 uninstall -y pillow
# RUN pip3 install "pillow<7"
RUN pip3 install ipython
RUN pip3 install seaborn
RUN pip3 install matplotlib
# RUN pip3 install "matplotlib>=3.2.2,<4"

# deep sort dependencies
RUN pip3 install filterpy
RUN pip3 install scikit-image
RUN pip3 install lap

# RUN apt-get install -y 	v4l-utils ffmpeg libyaml-cpp-dev libogg-dev libtheora-dev

# install sauvc dependencies
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_python-py36.so /usr/lib/aarch64-linux-gnu/libboost_python37.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_python-py36.a /usr/lib/aarch64-linux-gnu/libboost_python37.a

# RUN mkdir -p /additional_packages_ws/src
WORKDIR /additional_packages_ws/src
RUN git clone https://github.com/jinmenglei/serial.git
RUN git clone -b iron https://github.com/ros-perception/vision_opencv.git
RUN mv vision_opencv/cv_bridge/ .
RUN rm -r vision_opencv/
RUN git clone -b ros2 https://github.com/ros-drivers/zbar_ros.git
RUN apt-get install -y 	ffmpeg libyaml-cpp-dev
RUN git clone -b iron https://github.com/ros-perception/image_common.git
RUN apt-get install -y 	libogg-dev libtheora-dev
RUN git clone -b iron https://github.com/ros-perception/image_transport_plugins.git

WORKDIR /additional_packages_ws
RUN /bin/bash -c "source /opt/ros/iron/install/setup.bash && colcon build"
WORKDIR /additional_packages_ws/src
RUN git clone -b 0.7.0 https://github.com/ros-drivers/usb_cam.git
# RUN rosdep update
# RUN /bin/bash -c "source /additional_packages_ws/install/setup.bash &&  rosdep install --from-paths src --ignore-src -y"
WORKDIR /additional_packages_ws
RUN /bin/bash -c "source /opt/ros/iron/install/setup.bash && colcon build"

RUN echo 'source /additional_packages_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /welt_auv
CMD ["bash"]
