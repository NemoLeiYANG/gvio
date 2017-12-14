#!/bin/bash
set -e

cd /usr/local/src/
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir -p build
cd build
cmake ..
make
make install
