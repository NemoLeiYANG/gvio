#!/bin/bash
set -e  # exit on first error
OPENCV_URL=https://github.com/opencv/opencv/archive/3.2.0.zip

MACHINE_TYPE=`uname -m`
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
  DOWNLOAD_PATH=/usr/local/src
else
  DOWNLOAD_PATH=/mnt/sdcard
fi

install_dependencies() {
  apt-get -y install -qq \
    build-essential \
    cmake \
    git \
    libgtk2.0-dev \
    pkg-config \
    python-dev \
    python-numpy \
    libdc1394-22 \
    libdc1394-22-dev \
    libjpeg-dev \
    libpng12-dev \
    libtiff*-dev \
    libjasper-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libxine2-dev \
    libgstreamer0.10-dev \
    libgstreamer-plugins-base0.10-dev \
    libv4l-dev \
    libtbb-dev \
    libqt4-dev \
    libmp3lame-dev \
    libopencore-amrnb-dev \
    libopencore-amrwb-dev \
    libtheora-dev \
    libvorbis-dev \
    libxvidcore-dev \
    x264 \
    v4l-utils \
    unzip
}

download_opencv() {
  mkdir -p $DOWNLOAD_PATH
  cd $DOWNLOAD_PATH
  if [ ! -d opencv-3.2.0 ]; then
    wget --no-check-certificate $OPENCV_URL -O opencv3.2.0.zip
    unzip -qq opencv3.2.0.zip
    rm opencv3.2.0.zip
  fi
  cd -
}

install_opencv() {
  # compile and install opencv
  cd $DOWNLOAD_PATH/
  cd opencv-3.2.0
  mkdir -p build
  cd build
  cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_TBB=ON \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON ..
  make -j$(nproc)

  # THIS WILL CAUSE PROBLEMS WITH BUILDS THAT DEPEND ON OPENCV 2
  # THE REASON IS /usr/local/lib has precedence over /usr/lib
  make install
}

# MAIN
install_dependencies
download_opencv
install_opencv
