#!/bin/sh
set -e

# run clang
find . -name "*.cpp" | xargs clang-format-3.8 -i
find . -name "*.hpp" | xargs clang-format-3.8 -i

# find . -name *.cpp | xargs clang-tidy

MACHINE_TYPE=$(uname -m)
if [ "${MACHINE_TYPE}" = 'x86_64' ]; then
  CPU_COUNT=8
else
  CPU_COUNT=1
fi

# doxygen Doxyfile

mkdir -p build && cd build

cmake .. && time make -j${CPU_COUNT}

# ./experiments/kitti_runner /data/kitti/raw 2011_09_26 0046 ./experiments/configs/msckf_kitti_raw-2011_09_26-0046.yaml ./experiments
# python3 tests/scripts/plot_msckf.py ./experiments/msckf

# cd ../
# python3 scripts/msckf_tuner.py experiments/configs/kitti_raw_2011_09_26_0009.yaml
# cd ../ && bash scripts/test_runner.sh

# VALGRIND="valgrind --leak-check=full \
# 									 --show-leak-kinds=all \
# 									 --suppressions=../../.valgrind-suppression"

# ./tests/apriltag-mit_test
# ./tests/camera-camera_test
# ./tests/camera-camera_config_test
# ./tests/camera-ids_test
# ./tests/camera-pinhole_model_test
# ./tests/control-carrot_controller_test
# ./tests/control-pid_test
# ./tests/euroc-mav_dataset_test
# ./tests/feature2d-feature_container_test
# ./tests/feature2d-feature_test
# ./tests/feature2d-feature_track_test
# ./tests/feature2d-feature_tracker_test
# ./tests/feature2d-gms_matcher_test
# ./tests/feature2d-klt_tracker_test
# ./tests/feature2d-orb_tracker_test
# ./tests/gimbal-sbgc_test
# ./tests/imu-mpu6050_test
# ./tests/kitti-raw-calib_test
# ./tests/kitti-raw-oxts_test
# ./tests/kitti-raw-parse_test
# ./tests/kitti-raw-raw_test
# ./tests/msckf-camera_state_test
# ./tests/msckf-imu_state_test
# ./tests/msckf-feature_estimator_test
# ./tests/msckf-msckf_test
# ./tests/msckf-profiler_test
# ./tests/pwm-pca9685_test
# ./tests/quadrotor-quadrotor_model_test
# ./tests/quadrotor-mission_test
# ./tests/sim-carrot_controller_test
# ./tests/sim-camera_test
# ./tests/sim-twowheel_test
# ./tests/sim-world_test
# ./tests/util-config_test
# ./tests/util-data_test
# ./tests/util-file_test
# ./tests/util-gps_test
# ./tests/util-linalg_test
# ./tests/util-math_test
# ./tests/util-stats_test
# ./tests/util-time_test

# ./utils/pwm 40
# ./utils/imu configs/imu/config.yaml
# ./utils/camera configs/camera/ueye/cam1.yaml
# ./utils/camera configs/camera/ueye/cam2.yaml
# ./utils/camera configs/camera/ueye/cam3.yaml
./utils/recorder configs/recorder/config.yaml

# python3 scripts/plot_msckf.py /tmp/kitti_raw_2011_09_26_0005/run2/msckf
# python3 scripts/plot_euroc_mav_dataset.py /data/euroc_mav/raw/mav0
# python3 scripts/plot_feature_track.py /tmp/track.dat 1242 375
# python3 scripts/plot_feature_tracks.py /tmp/feature_tracks 1242 375
# python3 scripts/plot_twowheel.py /tmp/twowheel.dat
