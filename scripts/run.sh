#!/bin/sh
set -e

# run clang
find . -name "*.cpp" -print0 | xargs --null clang-format -i
find . -name "*.hpp" -print0 | xargs --null clang-format -i

# find . -name *.cpp | xargs clang-tidy

MACHINE_TYPE=$(uname -m)
if [ "${MACHINE_TYPE}" = 'x86_64' ]; then
  CPU_COUNT=8
else
  CPU_COUNT=1
fi

# doxygen Doxyfile

# python scripts/calibration/allan_variance.py \
#   --bag "/home/chutsu/gvio_datasets/calibration/imu_calibration/2018-03-11-14-42-05.bag" \
#   --imu_topic "/gvio/imu"

# BUILD
mkdir -p build && cd build
cmake .. && time make -j${CPU_COUNT}

# SCRIPTS
# python3 scripts/plot/plot_fig.py
# python3 scripts/maths/transforms.py; exit 0
# python3 ../scripts/plot/plot_landmarks.py /tmp/landmarks.csv /tmp/pose.csv; exit 0

# EXPERIMENTS
# cd experiments
# ./kitti_runner /data/kitti/raw 2011_09_26 0005 ./configs/msckf_kitti_raw-2011_09_26-0005.yaml /tmp/msckf
# ./kitti_runner /data/kitti/raw 2011_09_26 0046 ./configs/msckf_kitti_raw-2011_09_26-0046.yaml /tmp/msckf
# python3 scripts/plot_msckf.py /tmp/msckf/msckf; exit 0
# ./euroc_runner /data/euroc_mav/raw/V1_01_easy ./configs/msckf_euroc_V01_01_easy.yaml /tmp/msckf
# python3 scripts/plot_euroc_dataset.py /data/euroc_mav/raw/V1_01_easy
# ./sim_runner ./configs/sim.yaml ./configs/msckf_sim.yaml
# python3 scripts/plot_sim.py /tmp/sim
# ./tracker_benchmark /data/euroc_mav/raw/V1_01_easy; exit 0
# ./bundle_adjustment_benchmark /data/kitti/raw 2011_09_26 0005; exit 0

# TESTS
cd tests
# ./apriltag-mit_test
# ./camera-camera_test
# ./camera-camera_config_test
# ./camera-ids_test
# ./camera-pinhole_model_test
# ./control-carrot_controller_test
# ./control-pid_test
# ./dataset-euroc-mav_dataset_test
# ./dataset-euroc-camera_data_test
# ./dataset-euroc-imu_data_test
# ./dataset-euroc-ground_truth_test
# ./dataset-kitti-raw-calib_test
# ./dataset-kitti-raw-oxts_test
# ./dataset-kitti-raw-parse_test
# ./dataset-kitti-raw-raw_test
# ./feature2d-feature_container_test
# ./feature2d-feature_test
# ./feature2d-feature_track_test
# ./feature2d-feature_tracker_test
# ./feature2d-gms_matcher_test
# ./feature2d-grid_fast_test
# ./feature2d-klt_tracker_test
# ./feature2d-orb_tracker_test
# ./feature2d-stereo_orb_tracker_test
# ./feature2d-stereo_klt_tracker_test
# ./calibration-aprilgrid_test
# ./calibration-chessboard_test
# ./calibration-calib_params_test
# ./calibration-calib_preprocessor_test
# ./calibration-calib_validator_test
# ./calibration-stereo_calib_test
# ./calibration-gimbal_calib_test
# ./calibration-residual_test
# ./gimbal-gmr-gmr_test
# ./gimbal-sbgc_test
# ./gimbal-saliency_test
# ./imu-mpu6050_test
# ./msckf-camera_state_test
# ./msckf-imu_state_test
./msckf-feature_estimator_test
# ./msckf-msckf_test
# ./pwm-pca9685_test
# ./quadrotor-quadrotor_model_test
# ./quadrotor-mission_test
# ./sim-bezier_test
# ./sim-camera_test
# ./sim-camera_motion_test
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

# TEST SCRIPTS
# python3 scripts/plot_msckf.py /tmp/msckf/msckf
# python3 scripts/plot_euroc_mav_dataset.py /data/euroc_mav/raw/mav0
# python3 scripts/plot_feature_track.py /tmp/track.dat 1242 375
# python3 scripts/plot_feature_tracks.py /tmp/feature_tracks 1242 375
# python3 scripts/plot_twowheel.py /tmp/twowheel.dat
# python3 scripts/plot_sim.py /tmp/sim

# TOOLS
# ./tools/pwm 40
# ./tools/imu configs/imu/config.yaml
# ./tools/camera configs/camera/ueye/cam1.yaml
# ./tools/camera configs/camera/ueye/cam2.yaml
# ./tools/camera configs/camera/ueye/cam3.yaml
