#!/bin/sh
set -e

# run clang
find . -name "*.cpp" | xargs clang-format-3.8 -i
find . -name "*.hpp" | xargs clang-format-3.8 -i

# find . -name *.cpp | xargs clang-tidy

doxygen Doxyfile

mkdir -p build && cd build

cmake .. && time make -j8

# cd ../ && bash scripts/test_runner.sh

# VALGRIND="valgrind --leak-check=full \
# 									 --show-leak-kinds=all \
# 									 --suppressions=../../.valgrind-suppression"

cd tests
# ./apriltag-mit_test
# ./camera-camera_test
# ./camera-camera_config_test
# ./camera-ids_test
# ./camera-pinhole_model_test
# ./feature2d-feature_container_test
# ./feature2d-feature_test
# ./feature2d-feature_track_test
# ./feature2d-feature_tracker_test
# ./feature2d-gms_matcher_test
# ./feature2d-klt_tracker_test
# ./feature2d-orb_tracker_test
# ./imu-mpu6050_test
# ./kitti-raw-calib_test
# ./kitti-raw-oxts_test
# ./kitti-raw-parse_test
# ./kitti-raw-raw_test
./msckf-camera_state_test
./msckf-imu_state_test
# ./msckf-feature_estimator_test
# ./msckf-msckf_test
# ./msckf-profiler_test
# ./util-config_test
# ./util-data_test
# ./util-file_test
# ./util-gps_test
# ./util-linalg_test
# ./util-math_test
# ./util-stats_test
# ./util-time_test

# python3 scripts/plot_msckf.py
# python3 scripts/plot_feature_track.py /tmp/track.dat 1242 375
# python3 scripts/plot_feature_tracks.py /tmp/feature_tracks 1242 375
