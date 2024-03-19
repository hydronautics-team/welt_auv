FROM dustynv/ros:iron-ros-core-l4t-r32.7.1

RUN apt update 
RUN apt install -y --no-install-recommends git wget

RUN apt update && apt install -y --no-install-recommends git wget libboost-system-dev python3-pip software-properties-common curl graphviz graphviz-dev
RUN pip3 install pyserial transitions[diagrams]

# install pytorch
RUN wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl
RUN apt install -y python3-pip libopenblas-base libopenmpi-dev libomp-dev
RUN pip3 install Cython
RUN python3 -m pip install numpy torch-1.10.0-cp36-cp36m-linux_aarch64.whl

# install torchvision
RUN apt install -y libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
RUN git clone --branch v0.11.1 https://github.com/pytorch/vision torchvision
WORKDIR /torchvision
RUN export BUILD_VERSION=0.11.1
RUN ls
RUN python3 setup.py install --user

# install sauvc dependencies
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_python-py36.so /usr/lib/aarch64-linux-gnu/libboost_python37.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_python-py36.a /usr/lib/aarch64-linux-gnu/libboost_python37.a

RUN pip3 install "numpy>=1.18.5,<1.20"
RUN pip3 install tqdm
RUN pip3 install pandas==1.1.0
RUN apt-get install -y python3-requests build-essential libatlas-base-dev gfortran libfreetype6-dev
RUN pip3 install scipy
RUN pip3 uninstall -y pillow
RUN pip3 install "pillow<7"
RUN pip3 install seaborn
RUN pip3 install "matplotlib>=3.2.2,<4"

RUN mkdir -p /ws/additional_packages_ws/src
WORKDIR /ws/additional_packages_ws/src
RUN git clone https://github.com/jinmenglei/serial.git
RUN git clone -b iron https://github.com/ros-perception/vision_opencv.git
RUN git clone -b iron https://github.com/ros-perception/image_common.git
RUN git clone -b ros2 https://github.com/ros-drivers/usb_cam.git
RUN git clone -b iron https://github.com/ros-perception/image_pipeline.git
RUN mv image_pipeline/image_view/ .
RUN rm -r image_pipeline/

RUN apt-get install -y libzbar-dev
RUN git clone -b ros2 https://github.com/ros-drivers/zbar_ros.git

# deep sort dependencies
RUN pip3 install filterpy==1.4.5
RUN pip3 install scikit-image==0.17.2
RUN pip3 install lap==0.4.0

RUN /bin/bash -c "source /opt/ros/iron/setup.bash && cd /additional_packages_ws && colcon build"

RUN echo 'source /additional_packages_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /welt_auv
CMD ["bash"]
