#!/bin/bash
set -e  # exit on first error
UBUNTU_VERSION=`lsb_release --release | cut -f2`
DOWNLOAD_PATH="/usr/local/src/"

MACHINE_TYPE=`uname -m`
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
  DOWNLOAD_PATH=/usr/local/src
else
  DOWNLOAD_PATH=/mnt/sdcard
fi

install_dependencies() {
    apt-get update -qq
    apt-get install -qq -y cmake \
                           libgoogle-glog-dev \
                           libatlas-base-dev \
                           libeigen3-dev \
                           libsuitesparse-dev
}

install_suitesparse_fix() {
    add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687 -y
    apt-get update -qq
    apt-get install -qq libsuitesparse-dev
}

install_ceres_solver() {
    # clone ceres solver
    mkdir -p $DOWNLOAD_PATH
    cd $DOWNLOAD_PATH
    git clone https://github.com/ceres-solver/ceres-solver.git

    # go into ceres-solver repo and prepare for build
    cd ceres-solver
    mkdir build
    cd build
    cmake ..

    # compile and install
    make
    # make test
    make install
}


# MAIN
if [ $UBUNTU_VERSION == "16.04" ]; then
    install_dependencies
    install_ceres_solver

elif [ $UBUNTU_VERSION == "14.04" ]; then
    install_dependencies
    install_suitesparse_fix
    install_ceres_solver
fi
