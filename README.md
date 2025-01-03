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
    coinor-libipopt-dev \
 && apt-get clean

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
WORKDIR /home/user/catkin_ws/
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

WORKDIR /home/user/catkin_ws/src/
RUN mv /home/user/catkin_ws/src/gb_visual_detection_3d /home/user/

#modify IpSmartPtr.hpp . if you do not run, you can`t use mpc_ros and you can`t catkin_make
WORKDIR /usr/include/coin  
RUN rm -rf IpSmartPtr.hpp \
    && git clone https://github.com/mulempyo/file.git \
    && mv /usr/include/coin/file/IpSmartPtr.hpp /usr/include/coin \
    && rm -rf file

WORKDIR /home/user/catkin_ws/
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make && source ./devel/setup.bash" \
    && mv /home/user/gb_visual_detection_3d /home/user/catkin_ws/src/ 

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
