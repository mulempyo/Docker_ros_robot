<H1>Dockerfile Download</H1>

<H2>Step1</H2>


``` docker pull nvcr.io/nvidia/l4t-base:r32.4.4 ```

``` docker pull ros:melodic ```


<H2>Step2</H2>
<H3> Clone docker file (Linux/arm64)</H3>

``````````````````````
#In linux/arm64 dockerfile, "nvcr.io/nvidia/l4t-base:r32.4.4" is jetpack-4.3 docker image
FROM nvcr.io/nvidia/l4t-base:r32.4.4
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
    unzip \
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
    gfortran \
    usbutils  \
    imagemagick \
    libblas-dev \
    liblapack-dev
    
# melodic: Dependencies for building packages
WORKDIR /home/user/
RUN apt install -y ca-certificates && update-ca-certificates\
    && wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
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
    && apt-get install -y ros-melodic-base-local-planner \
    && apt-get install -y ros-melodic-roslint \
    && apt-get install -y ros-melodic-xacro \
    && apt-get install -y ros-melodic-pcl-ros \
    && apt-get install -y ros-melodic-gazebo-ros-pkgs 

# my docker ros robot github download
WORKDIR /home/user/catkin_ws
RUN git clone https://github.com/mulempyo/Docker_ros_robot.git \
    && mv Docker_ros_robot src \
    && cd src && rm -rf YDLidar-SDK ydlidar_ros_driver myahrs_driver realsense-ros  

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
    && cd /home/user/catkin_ws/src/ && git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git   

#yolov3-tiny.weights downloads
WORKDIR /home/user/catkin_ws/src/
RUN  wget pjreddie.com/media/files/yolov3-tiny.weights \
     && mv /home/user/catkin_ws/src/yolov3-tiny.weights /home/user/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/

#move the directories because I want to resolve the catkin_make error
WORKDIR /home/user/catkin_ws/src/
RUN mv /home/user/catkin_ws/src/gb_visual_detection_3d /home/user/ \
    && mv /home/user/catkin_ws/src/detect_object /home/user/

#cmake-3.18 download
WORKDIR /home/user/
RUN wget https://github.com/Kitware/CMake/releases/download/v3.18.0/cmake-3.18.0.tar.gz \
    && tar -xvzf cmake-3.18.0.tar.gz \
    && cd cmake-3.18.0 && mkdir build && cd build && ../bootstrap && make -j$(nproc) && sudo make install

#g2o download    
WORKDIR /home/user/    
RUN sudo apt update \
    && sudo apt install -y software-properties-common \
    && sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y \
    && sudo apt update \
    && sudo apt install -y gcc-9 g++-9 libeigen3-dev libqt5core5a libqt5gui5 libqt5widgets5 qtbase5-dev libqglviewer-dev-qt5 libsuitesparse-dev \
    && sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 \
    && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100 \
    && git clone https://github.com/RainerKuemmerle/g2o \
    && cd /home/user/g2o && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_CHOLMOD=ON -DG2O_USE_CSPARSE=ON -DG2O_USE_OPENGL=OFF -DG2O_USE_OPENMP=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS="-std=c++17 -lstdc++fs" \
    && make -j$(nproc) \
    && sudo make install    
    
#casadi download for mpc_ros    
WORKDIR /home/user/catkin_ws/src/
RUN cd /home/user/catkin_ws/src/navigation/mpc_ros/include/ && rm -rf casadi \
    && cd /home/user/catkin_ws/src/ && git clone https://github.com/casadi/casadi.git \
    && cd /home/user/catkin_ws/src/casadi/ && mkdir build && cd build \
    && cmake .. -DWITH_IPOPT=ON && make -j$(nproc) && sudo make install \
    && mv /home/user/catkin_ws/src/casadi /home/user/catkin_ws/src/navigation/mpc_ros/include/

#arduino download
WORKDIR /home/user/
RUN wget https://downloads.arduino.cc/arduino-1.8.19-linuxaarch64.tar.xz \
    && tar -xvf arduino-1.8.19-linuxaarch64.tar.xz \
    && cd arduino-1.8.19 && ./install.sh && cd libraries && rm -rf ros_lib \
    && cd /home/user/arduino-1.8.19/libraries && /bin/bash -c "source /opt/ros/melodic/setup.bash && rosrun rosserial_arduino make_libraries.py ."  

WORKDIR /home/user/
ENV PATH="/usr/local/cuda-10.2/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-10.2/lib64:${LD_LIBRARY_PATH}"

WORKDIR /home/user/
RUN sudo apt update \
    && sudo apt install -y gcc-7 g++-7 \
    && sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 \
    && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 100 \
    && sudo update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-7 100

RUN update-alternatives --set gcc /usr/bin/gcc-7 \
    && update-alternatives --set g++ /usr/bin/g++-7 \
    && update-alternatives --set cpp /usr/bin/cpp-7

# realsense driver download    
WORKDIR /home/user/
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.tar.gz \
    && tar -xvzf v2.50.0.tar.gz \
    && cd librealsense-2.50.0 && mkdir build && cd build && cmake .. -DBUILD_WITH_CUDA=true -DCMAKE_CUDA_ARCHITECTURES=53 && make -j1 && sudo make install
    
# realsense-ros download
WORKDIR /home/user/catkin_ws/src/
RUN git clone https://github.com/IntelRealSense/realsense-ros.git \
    && cd realsense-ros && git checkout 2.3.2 

WORKDIR /home/user/catkin_ws/src/   
RUN mv ./darknet_ros /home/user/ && mv ./realsense-ros /home/user/ 

WORKDIR /home/user/
RUN sudo apt update \
    && sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 \
    && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100 \
    && sudo update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-9 100
    
RUN update-alternatives --set gcc /usr/bin/gcc-9 \
    && update-alternatives --set g++ /usr/bin/g++-9 \
    && update-alternatives --set cpp /usr/bin/cpp-9

#WORKDIR /home/user/catkin_ws/
#RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=release -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS="-std=c++17 -lstdc++fs" && source ./devel/setup.bash"

## if you start image, follow next step ##

## WORKDIR /home/user/catkin_ws/src/detect_object
#RUN gcc-8,g++-8 version -> nvcc -arch=sm_53 -o test_detect_kernel.o -c src/detect_object_kernel.cu

#update-alternatives --config gcc
#update-alternatives --config g++


#move the directories again 
#WORKDIR /home/user/catkin_ws/
#RUN gedit ~/.bashrc \
#    && export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}} \
#    && export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
#    && source ~/.bashrc \
#    && source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=release -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS="-std=c++17 -lstdc++fs" 
#    && source ./devel/setup.bash \
#    && mv /home/user/gb_visual_detection_3d /home/user/catkin_ws/src/ \
#    && mv /home/user/detect_object /home/user/catkin_ws/src/ \
#    && source /opt/ros/melodic/setup.bash && catkin_make && source ./devel/setup.bash

# cuDNN 버전 확인:
#/sbin/ldconfig -N -v $(sed 's/:/ /' <<< $LD_LIBRARY_PATH) 2>/dev/null | grep libcudnn
# >> 	libcudnn.so.7 -> libcudnn.so.7.6.5

#cuda 확인:
#nvcc --version
#>>  nvcc: NVIDIA (R) Cuda compiler driver
#    Copyright (c) 2005-2019 NVIDIA Corporation
#    Built on Wed_Oct_23_19:24:38_PDT_2019
#    Cuda compilation tools, release 10.2, V10.2.89  

#if you do not install fw:5.13.0
#WORKDIR /home/user/
#RUN wget -O firmware.zip https://www.intelrealsense.com/download/19295/?_ga=2.65911659.471808776.1735634418-738563781.1729084886 \
#    && unzip firmware.zip \
#    #connect hardware
#    && rs-fw-update -f Signed_Image_UVC_5_13_0_50.bin 
#    #Signed_Image_UVC_5_13_0_50.bin file  in D400_Series_FW_5_13_0_50

CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && bash"]

``````````````````````
<H3> Clone docker file (Linux/amd64)</H3>

``````````````````````
#In linux/amd64 environment dockerfile, i have nvidia geforce MX570 A
FROM ros:melodic
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
    unzip \
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
    gfortran \
    usbutils  \
    imagemagick \
    libblas-dev \
    liblapack-dev
    
# melodic: Dependencies for building packages
WORKDIR /home/user/
RUN apt update \
    && apt install -y --no-install-recommends \
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
    && apt-get install -y ros-melodic-base-local-planner \
    && apt-get install -y ros-melodic-roslint \
    && apt-get install -y ros-melodic-xacro \
    && apt-get install -y ros-melodic-pcl-ros \
    && apt-get install -y ros-melodic-gazebo-ros-pkgs \
    && apt-get install -y ros-melodic-rviz
    

# my docker ros robot github download
WORKDIR /home/user/catkin_ws
RUN git clone https://github.com/mulempyo/Docker_ros_robot.git \
    && mv Docker_ros_robot src \
    && cd src && rm -rf YDLidar-SDK ydlidar_ros_driver myahrs_driver realsense-ros  

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
    && cd /home/user/catkin_ws/src/ && git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
    
#yolov3-tiny.weights downloads
WORKDIR /home/user/catkin_ws/src/
RUN  wget pjreddie.com/media/files/yolov3-tiny.weights \
     && mv /home/user/catkin_ws/src/yolov3-tiny.weights /home/user/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/

#move the directories because I want to resolve the catkin_make error
WORKDIR /home/user/catkin_ws/src/
RUN mv /home/user/catkin_ws/src/gb_visual_detection_3d /home/user/ \
    && mv /home/user/catkin_ws/src/detect_object /home/user/

#cmake-3.18 download
WORKDIR /home/user/
RUN wget https://github.com/Kitware/CMake/releases/download/v3.18.0/cmake-3.18.0.tar.gz \
    && tar -xvzf cmake-3.18.0.tar.gz \
    && cd cmake-3.18.0 && mkdir build && cd build && ../bootstrap && make -j$(nproc) && sudo make install

#g2o download    
WORKDIR /home/user/    
RUN sudo apt update \
    && sudo apt install -y software-properties-common \
    && sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y \
    && sudo apt update \
    && sudo apt install -y gcc-9 g++-9 libeigen3-dev libqt5core5a libqt5gui5 libqt5widgets5 qtbase5-dev libqglviewer-dev-qt5 libsuitesparse-dev \
    && sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 \
    && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100 \
    && git clone https://github.com/RainerKuemmerle/g2o \
    && cd /home/user/g2o && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_CHOLMOD=ON -DG2O_USE_CSPARSE=ON -DG2O_USE_OPENGL=OFF -DG2O_USE_OPENMP=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS="-std=c++17 -lstdc++fs" \
    && make -j$(nproc) \
    && sudo make install    

WORKDIR /home/user/
RUN sudo apt update \
    && sudo apt install -y gcc-7 g++-7 \
    && sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 \
    && sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 100 \
    && sudo update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-7 100
    
RUN update-alternatives --set gcc /usr/bin/gcc-7 \
    && update-alternatives --set g++ /usr/bin/g++-7 \
    && update-alternatives --set cpp /usr/bin/cpp-7    

#casadi download for mpc_ros    
WORKDIR /home/user/catkin_ws/src/
RUN cd /home/user/catkin_ws/src/navigation/mpc_ros/include/ && rm -rf casadi \
    && cd /home/user/catkin_ws/src/ && git clone https://github.com/casadi/casadi.git \
    && cd /home/user/catkin_ws/src/casadi/ && mkdir build && cd build \
    && cmake .. -DWITH_IPOPT=ON && make -j$(nproc) && sudo make install \
    && mv /home/user/catkin_ws/src/casadi /home/user/catkin_ws/src/navigation/mpc_ros/include/

# Install CUDA 10.2 Toolkit
WORKDIR /home/user/
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends ca-certificates && sudo update-ca-certificates \ 
    && wget --no-check-certificate https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin \
    && mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && wget --no-check-certificate https://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb \
    && dpkg -i cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb \
    && apt-key add /var/cuda-repo-10-2-local-10.2.89-440.33.01/7fa2af80.pub \
    && apt-get update \
    && apt-get install -y cuda-toolkit-10-2 

WORKDIR /home/user/
ENV PATH="/usr/local/cuda-10.2/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-10.2/lib64:${LD_LIBRARY_PATH}"

# realsense driver download    
WORKDIR /home/user/
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.tar.gz \
    && tar -xvzf v2.50.0.tar.gz \
    && cd librealsense-2.50.0 && mkdir build && cd build &&  cmake .. -DBUILD_WITH_CUDA=true -DCMAKE_CUDA_ARCHITECTURES=75 && make -j1 && sudo make install
    
#realsense-ros download
WORKDIR /home/user/catkin_ws/
RUN mkdir src && cd src && git clone https://github.com/IntelRealSense/realsense-ros.git \
    && cd realsense-ros && git checkout 2.3.2 


## if you start image, follow next step ##
    
# Install cuDNN 7.6.5 (Debian package version) runtime download first, and then dev download
#WORKDIR /home/user/
#RUN wget "cudnn7.6.5 runtime download link" \
#    && mv "cudnn7.6.5 download link" libcudnn7_7.6.5.32-1+cuda10.2_amd64.deb \ 
#    && sudo dpkg -i libcudnn7_7.6.5.32-1+cuda10.2_amd64.deb \
#    && wget "cudnn7.6.5-dev download link" \
#    && mv "cudnn7.6.5-dev download link" libcudnn7-dev_7.6.5.32-1+cuda10.2_amd64.deb \
#    && sudo dpkg -i libcudnn7-dev_7.6.5.32-1+cuda10.2_amd64.deb \
#    && apt-get update \
#    && gedit ~/.bashrc
#    && export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}} \
#    && export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
#    && source ~/.bashrc
#    && apt-get update 

## gcc8,g++8 version, -> nvcc -arch=sm_75 -o test_detect_kernel.o -c src/detect_object_kernel.cu

#move the directories again.
#WORKDIR /home/user/catkin_ws/
#RUN source /opt/ros/melodic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=release && source ./devel/setup.bash \
#    && mv /home/user/gb_visual_detection_3d /home/user/catkin_ws/src/ \
#    && mv /home/user/detect_object /home/user/catkin_ws/src/ \
#    && source /opt/ros/melodic/setup.bash && catkin_make && source ./devel/setup.bash

# cuDNN 버전 확인:
#/sbin/ldconfig -N -v $(sed 's/:/ /' <<< $LD_LIBRARY_PATH) 2>/dev/null | grep libcudnn
# >> 	libcudnn.so.7 -> libcudnn.so.7.6.5

#cuda 확인:
#nvcc --version
#>>  nvcc: NVIDIA (R) Cuda compiler driver
#    Copyright (c) 2005-2019 NVIDIA Corporation
#    Built on Wed_Oct_23_19:24:38_PDT_2019
#    Cuda compilation tools, release 10.2, V10.2.89  


CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && bash"]

``````````````````````

<H2>Step3</H2>

<H3>BUILD</H3>

```docker build --platform linux/amd64 -t <hub_name>/<image_name>:<tag> --push . ```

```docker build --platform linux/arm64 -t <hub_name>/<image_name>:<tag> --push . ```

<H3>RUN</H3>
<H4>In window computer, use XLaunch</H4>

``` docker run --name <container_name> --env=DISPLAY=host.docker.internal:0 --volume="C:\\"/mnt/c" --restart=no --gpus all  --network=host -t -d <hub_name>/<image_name>:<tag> ```

<H4>In Jetson Nano</H4>

``` sudo docker run -it --privileged --device=/dev/ttyACM1 --device=/dev/ttyUSB0 --runtime nvidia --env=DISPLAY=$DISPLAY --volume=/tmp/.X11-unix:/tmp/.X11-unix --network=host -t <hub_name>/<image_name>:<tag> ```
