#!/bin/bash
ROS_VERSION=kinetic
CATKIN_WS=$HOME/kalibr_ws

sudo apt-get install -qq -y \
    python-setuptools \
    python-rosinstall \
    ipython \
    libeigen3-dev \
    libboost-all-dev \
    doxygen \
    libopencv-dev \
    python-software-properties \
    software-properties-common \
    libpoco-dev \
    python-matplotlib \
    python-scipy \
    python-git \
    python-pip \
    ipython libtbb-dev \
    libblas-dev \
    liblapack-dev \
    python-catkin-tools \
    libv4l-dev \
    ros-$ROS_VERSION-vision-opencv \
    ros-$ROS_VERSION-image-transport-plugins \
    ros-$ROS_VERSION-cmake-modules

sudo pip install python-igraph --upgrade

# Create Kalibr catkin workspace
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init

# Clone Kalibr
cd src
if [ ! -d Kalibr ]; then
    git clone https://github.com/ethz-asl/Kalibr.git
fi

# Build Kalibr
cd $CATKIN_WS
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
