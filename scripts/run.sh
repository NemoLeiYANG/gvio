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

cd tests
# ./apriltag-mit_test
# ./camera-camera_test
# ./camera-camera_config_test
# ./camera-ids_test
# ./camera-pinhole_model_test
# ./control-carrot_controller_test
# ./control-pid_test
# ./dataset-euroc-mav_dataset_test
# ./dataset-kitti-raw-calib_test
# ./dataset-kitti-raw-oxts_test
# ./dataset-kitti-raw-parse_test
# ./dataset-kitti-raw-raw_test
# ./feature2d-feature_container_test
# ./feature2d-feature_test
# ./feature2d-feature_track_test
# ./feature2d-feature_tracker_test
# ./feature2d-gms_matcher_test
# ./feature2d-klt_tracker_test
# ./feature2d-orb_tracker_test
# ./gimbal-calibration-aprilgrid_test
# ./gimbal-calibration-chessboard_test
# ./gimbal-calibration-calib_params_test
# ./gimbal-calibration-calib_preprocessor_test
# ./gimbal-calibration-calib_validator_test
# ./gimbal-calibration-gimbal_calib_test
# ./gimbal-calibration-residual_test
# ./gimbal-gmr-gmr_test
# ./gimbal-sbgc_test
# ./gimbal-saliency_test
# ./imu-mpu6050_test
# ./msckf-camera_state_test
# ./msckf-imu_state_test
# ./msckf-feature_estimator_test
# ./msckf-msckf_test
# ./msckf-profiler_test
# ./pwm-pca9685_test
# ./quadrotor-quadrotor_model_test
# ./quadrotor-mission_test
# ./sim-carrot_controller_test
# ./sim-camera_test
# ./sim-twowheel_test
# ./sim-world_test
# ./util-config_test
# ./util-data_test
# ./util-file_test
# ./util-gps_test
# ./util-linalg_test
# ./util-math_test
# ./util-stats_test
# ./util-time_test

# ./utils/pwm 40
# ./utils/imu configs/imu/config.yaml
# ./utils/camera configs/camera/ueye/cam1.yaml
# ./utils/camera configs/camera/ueye/cam2.yaml
# ./utils/camera configs/camera/ueye/cam3.yaml
# rm -rf /mnt/sdcard/gvio_recording
# ./utils/recorder configs/recorder/config.yaml

# python3 scripts/plot_msckf.py /tmp/kitti_raw_2011_09_26_0005/run2/msckf
# python3 scripts/plot_euroc_mav_dataset.py /data/euroc_mav/raw/mav0
# python3 scripts/plot_feature_track.py /tmp/track.dat 1242 375
# python3 scripts/plot_feature_tracks.py /tmp/feature_tracks 1242 375
# python3 scripts/plot_twowheel.py /tmp/twowheel.dat
