CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(gvio)

# C++ COMPILER SETTINGS
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(BUILD_SHARED_LIBS ON)
SET(CMAKE_BUILD_TYPE RELEASE)
# SET(CMAKE_BUILD_TYPE DEBUG)
# SET(CMAKE_IGNORE_PATH `catkin locate`)

# DEPENDENCIES
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(SuiteSparse REQUIRED)
INCLUDE(cmake/ImportEigen3.cmake)

# INCLUDES
INCLUDE_DIRECTORIES(include /usr/include/eigen3 ${SUITESPARSE_INCLUDE_DIRS})

# LIBRARY
FILE(COPY configs DESTINATION ${PROJECT_BINARY_DIR})
ADD_LIBRARY(
  ${PROJECT_NAME}
  # apriltag
  src/apriltag/base_detector.cpp
  src/apriltag/mit.cpp
  src/apriltag/michigan.cpp
  # control
  src/control/carrot_controller.cpp
  src/control/pid.cpp
  # camera
  src/camera/camera.cpp
  src/camera/camera_config.cpp
  src/camera/ids.cpp
  src/camera/distortion.cpp
  src/camera/pinhole_model.cpp
  # dataset
  src/dataset/euroc/camera_data.cpp
  src/dataset/euroc/ground_truth.cpp
  src/dataset/euroc/imu_data.cpp
  src/dataset/euroc/mav_dataset.cpp
  src/dataset/kitti/raw/calib.cpp
  src/dataset/kitti/raw/oxts.cpp
  src/dataset/kitti/raw/parse.cpp
  src/dataset/kitti/raw/raw.cpp
  # driver
  src/driver/i2c.cpp
  src/driver/uart.cpp
  # msckf
  src/msckf/blackbox.cpp
  src/msckf/camera_state.cpp
  src/msckf/feature_estimator.cpp
  src/msckf/imu_state.cpp
  src/msckf/msckf.cpp
  # feature
  src/feature2d/draw.cpp
  src/feature2d/feature.cpp
  src/feature2d/feature_container.cpp
  src/feature2d/feature_track.cpp
  src/feature2d/feature_tracker.cpp
  src/feature2d/gms_matcher.cpp
  src/feature2d/orb_tracker.cpp
  src/feature2d/klt_tracker.cpp
  src/feature2d/stereo_tracker.cpp
  # gimbal
  src/gimbal/calibration/aprilgrid.cpp
  src/gimbal/calibration/calib_data.cpp
  src/gimbal/calibration/calib_params.cpp
  src/gimbal/calibration/calib_preprocessor.cpp
  src/gimbal/calibration/calib_validator.cpp
  src/gimbal/calibration/camchain.cpp
  src/gimbal/calibration/camera_property.cpp
  src/gimbal/calibration/chessboard.cpp
  src/gimbal/calibration/gimbal_calib.cpp
  src/gimbal/calibration/residual.cpp
  src/gimbal/calibration/stereo_calib.cpp
  src/gimbal/gmr/gmr.cpp
  src/gimbal/gmr/slic.cpp
  src/gimbal/gimbal.cpp
  src/gimbal/gimbal_model.cpp
  # src/gimbal/saliency.cpp
  src/gimbal/sbgc.cpp
  # imu
  src/imu/imu_base.cpp
  src/imu/mpu6050.cpp
  # pwm
  src/pwm/pca9685.cpp
  # quadrotor
  src/quadrotor/control/attitude_controller.cpp
  src/quadrotor/control/position_controller.cpp
  src/quadrotor/control/waypoint_controller.cpp
  src/quadrotor/mission.cpp
  src/quadrotor/quadrotor_model.cpp
  # quaternion
  src/quaternion/jpl.cpp
  # sim
  src/sim/camera.cpp
  src/sim/camera_motion.cpp
  src/sim/twowheel.cpp
  src/sim/world.cpp
  # utils
  src/util/config.cpp
  src/util/data.cpp
  src/util/euler.cpp
  src/util/file.cpp
  src/util/gps.cpp
  src/util/linalg.cpp
  src/util/math.cpp
  src/util/vision.cpp
  src/util/stats.cpp
  src/util/time.cpp
)
SET(
  ${PROJECT_NAME}_DEPS
  ${OpenCV_LIBS}
  ${SUITESPARSE_LIBRARIES}
  apriltags_mit
  apriltag
  yaml-cpp
  pthread
  ceres
  ueye_api
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
INSTALL(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
INSTALL(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/scripts DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_configs DESTINATION ${PROJECT_BINARY_DIR}/tests)
SET(
  UNITTESTS
  # apriltag-mit_test
  # camera-camera_test
  # camera-camera_config_test
  # camera-ids_test
  # camera-pinhole_model_test
  # control-carrot_controller_test
  # control-pid_test
  # dataset-euroc-camera_data_test
  # dataset-euroc-ground_truth_test
  # dataset-euroc-imu_data_test
  # dataset-euroc-mav_dataset_test
  # dataset-kitti-raw-calib_test
  # dataset-kitti-raw-oxts_test
  # dataset-kitti-raw-parse_test
  # dataset-kitti-raw-raw_test
  # feature2d-feature_container_test
  # feature2d-feature_test
  # feature2d-feature_track_test
  feature2d-feature_tracker_test
  # feature2d-gms_matcher_test
  # feature2d-klt_tracker_test
  # feature2d-orb_tracker_test
  # feature2d-stereo_tracker_test
  # gimbal-calibration-aprilgrid_test
  # gimbal-calibration-calib_data_test
  # gimbal-calibration-calib_params_test
  # gimbal-calibration-calib_preprocessor_test
  # gimbal-calibration-calib_validator_test
  # gimbal-calibration-chessboard_test
  # gimbal-calibration-gimbal_calib_test
  # gimbal-calibration-residual_test
  # gimbal-calibration-stereo_calib_test
  # gimbal-gmr-gmr_test
  # gimbal-sbgc_test
  # # gimbal-saliency_test
  # imu-mpu6050_test
  # msckf-camera_state_test
  # msckf-feature_estimator_test
  # msckf-imu_state_test
  # msckf-msckf_test
  # pwm-pca9685_test
  # quadrotor-quadrotor_model_test
  # quadrotor-mission_test
  # sim-bezier_test
  # sim-camera_test
  # sim-camera_motion_test
  # sim-twowheel_test
  sim-world_test
  # util-config_test
  # util-data_test
  # util-file_test
  # util-gps_test
  # util-linalg_test
  # util-math_test
  # util-stats_test
  # util-time_test
)
FOREACH(TEST ${UNITTESTS})
  STRING(REGEX REPLACE "-" "/" TEST_PATH ${TEST})
  ADD_EXECUTABLE(${TEST} tests/${TEST_PATH}.cpp)
  TARGET_LINK_LIBRARIES(${TEST} ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
ENDFOREACH(TEST)

# EXPERIMENTS
FILE(COPY experiments/configs DESTINATION ${PROJECT_BINARY_DIR}/experiments)
FILE(COPY experiments/scripts DESTINATION ${PROJECT_BINARY_DIR}/experiments)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/experiments)
SET(EXPERIMENTS kitti_runner euroc_runner sim_runner)
FOREACH(TEST ${EXPERIMENTS})
  STRING(REGEX REPLACE "-" "/" TEST_PATH ${TEST})
  ADD_EXECUTABLE(${TEST} experiments/${TEST_PATH}.cpp)
  TARGET_LINK_LIBRARIES(${TEST} ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
ENDFOREACH(TEST)

# # TOOLS
# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tools)
# SET(TOOLS
#     imu
#     camera
#     pwm
#     recorder)
# FOREACH(TOOL ${TOOLS})
#   STRING(REGEX REPLACE "-" "/" TOOL_PATH ${TOOL})
#   ADD_EXECUTABLE(${TOOL} tools/${TOOL_PATH}.cpp)
#   TARGET_LINK_LIBRARIES(${TOOL} ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
# ENDFOREACH(TOOL)
