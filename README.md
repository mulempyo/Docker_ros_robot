<H1>Dockerfile Download</H1>

<H2>Step1</H2>


``` docker pull nvidia/cuda:11.3.1-cudnn8-devel-ubuntu18.04 ```


<H2>Step2</H2>
<H3> Clone docker file</H3>

``````````````````````
# cuda,cudnn setting. first, docker pull nvidia/cuda:11.3.1-cudnn8-devel-ubuntu18.04
FROM nvidia/cuda:11.3.1-cudnn8-devel-ubuntu18.04
ARG DEBIAN_FRONTEND=noninteractive

USER root

# Update and install required packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    sudo \
    terminator \
    iproute2 \
    gedit \
    lsb-release \
    lsb-core \
    wget \
    nano \
    curl \
    gnupg2 \
    clinfo \
    zip \
    evince \
    libsdl2-dev \
    libsdl2-image-dev \
    coinor-libipopt-dev \
    libc6-dev \
    cmake \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    gdb \
    gfortran 

# melodic desktop full download
WORKDIR /home/user/
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt install -y curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt update \
    && apt install -y --no-install-recommends \
       ros-melodic-desktop-full \
       python-rosdep \
       python-rosinstall \
       python-rosinstall-generator \
       python-wstool \
       build-essential

RUN mkdir catkin_ws 

#ros pkg download
WORKDIR /home/user/catkin_ws/
RUN apt-get install -y ros-melodic-rosserial \ 
    && apt-get install -y ros-melodic-rosserial-python \
    && apt-get install -y ros-melodic-rosserial-arduino \
    && apt-get install -y ros-melodic-gmapping \
    && apt-get install -y ros-melodic-hector-slam \
    && apt-get install -y ros-melodic-teleop-twist-keyboard \
    && apt-get install -y ros-melodic-urdf \
    && apt-get install -y ros-melodic-joint-state-publisher \
    && apt-get install -y ros-melodic-robot-state-publisher \
    && apt-get install -y ros-melodic-robot-localization \
    && apt-get install -y ros-melodic-cartographer \
    && apt-get install -y ros-melodic-cartographer-ros \
    && apt-get install -y ros-melodic-ddynamic-reconfigure \
    && apt-get install -y ros-melodic-tf2-sensor-msgs \
    && apt-get install -y ros-melodic-move-base-msgs \
    && apt-get install -y ros-melodic-map-server \
    && apt-get install -y ros-melodic-base-local-planner 

# my docker ros robot github download
WORKDIR /home/user/catkin_ws
RUN git clone https://github.com/mulempyo/Docker_ros_robot.git \
    && mv Docker_ros_robot src \
    && cd src && rm -rf YDLidar-SDK ydlidar_ros_driver myahrs_driver   

# Imu driver download
WORKDIR /home/user/catkin_ws/src/
RUN git clone -b melodic-devel https://github.com/robotpilot/myahrs_driver.git    
    
# YDLidar driver download
WORKDIR /home/user/catkin_ws/src
RUN apt-get update \
    && apt install cmake pkg-config \
    && apt-get install -y python swig \
    && apt-get install -y python-pip \
    && git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd /home/user/catkin_ws/src/YDLidar-SDK/ && mkdir build \
    && cd /home/user/catkin_ws/src/YDLidar-SDK/build \
    && cmake .. && make && sudo make install \ 
    && git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git   

#yolov3-tiny.weights downloads
WORKDIR /home/user/catkin_ws/src/
RUN  wget pjreddie.com/media/files/yolov3-tiny.weights \
     && mv /home/user/catkin_ws/src/yolov3-tiny.weights /home/user/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/

#move the directories because I want to resolve the catkin_make error
WORKDIR /home/user/catkin_ws/src/
RUN mv /home/user/catkin_ws/src/gb_visual_detection_3d /home/user/ \
    && mv /home/user/catkin_ws/src/detect_object /home/user/

WORKDIR /home/user/
RUN wget https://github.com/Kitware/CMake/releases/download/v3.18.0/cmake-3.18.0.tar.gz \
    && tar -xvzf cmake-3.18.0.tar.gz \
    && cd cmake-3.18.0 && mkdir build && cd build && ../bootstrap && make -j$(nproc) && sudo make install
#casadi download for mpc_ros    
WORKDIR /home/user/catkin_ws/src/
RUN cd /home/user/catkin_ws/src/navigation/mpc_ros/include/ && rm -rf casadi \
    && cd /home/user/catkin_ws/src/ && git clone https://github.com/casadi/casadi.git \
    && cd /home/user/catkin_ws/src/casadi/ && mkdir build && cd build \
    && cmake .. -DWITH_IPOPT=ON && make -j$(nproc) && sudo make install \
    && mv /home/user/catkin_ws/src/casadi /home/user/catkin_ws/src/navigation/mpc_ros/include/

#move the directories again 
WORKDIR /home/user/catkin_ws/
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release && source ./devel/setup.bash" \
    mv /home/user/gb_visual_detection_3d /home/user/catkin_ws/src/ \
    && mv /home/user/detect_object /home/user/catkin_ws/src/ \
    && /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make && source ./devel/setup.bash" 

#arduino download
WORKDIR /home/user/
RUN wget https://downloads.arduino.cc/arduino-1.8.19-linuxaarch64.tar.xz \
    && tar -xvf arduino-1.8.19-linuxaarch64.tar.xz \
    && cd arduino-1.8.19 && ./install.sh && cd libraries && rm -rf ros_lib \
    && cd /home/user/arduino-1.8.19/libraries && /bin/bash -c "source /opt/ros/melodic/setup.bash && rosrun rosserial_arduino make_libraries.py ."  

#realsense driver download    
WORKDIR /home/user/
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.tar.gz \
    && tar -xvzf v2.50.0.tar.gz \
    && cd librealsense-2.50.0 && mkdir build && cd build &&  cmake .. -DBUILD_WITH_CUDA=true -DFORCE_RSUSB_BACKEND=true && make -j1 && sudo make install

#if you do not install fw:5.13.0
#WORKDIR /home/user/
#RUN wget -O firmware.zip https://www.intelrealsense.com/download/19295/?_ga=2.65911659.471808776.1735634418-738563781.1729084886 \
#    && unzip firmware.zip \
#    #connect hardware
#    && rs-fw-update -f Signed_Image_UVC_5_13_0_50.bin 
#    #Signed_Image_UVC_5_13_0_50.bin file  in D400_Series_FW_5_13_0_50

CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && bash"]

``````````````````````

<H2>Step3</H2>

<H3>BUILD</H3>

```docker buildx build --platform "linux/amd64,linux/arm64" -t <hub_name>/<image_name>:<tag> --push . ```

<H3>RUN</H3>
<H4>In window computer, use XLaunch</H4>

``` docker run --name <container_name> --env=DISPLAY=host.docker.internal:0 --volume="C:\\"/mnt/c" --restart=no --gpus all  --network=host -t -d <hub_name>/<image_name>:<tag> ```
