CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(gvio)

# C++ COMPILER SETTINGS
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE RELEASE)
# SET(CMAKE_BUILD_TYPE DEBUG)

# DEPENDENCIES
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
FIND_PACKAGE(OpenCV 3.0 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(SPQR REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED)
INCLUDE(cmake/ImportEigen3.cmake)

# INCLUDES
INCLUDE_DIRECTORIES(include /usr/include/eigen3 ${SPQR_INCLUDES})

# LIBRARY
ADD_LIBRARY(
  ${PROJECT_NAME}
  # apriltag
  src/apriltag/base_detector.cpp
  src/apriltag/mit.cpp
  src/apriltag/michigan.cpp
  # ceres
  src/ceres/jpl_quaternion_parameterization.cpp
  # camera
  src/camera/camera.cpp
  src/camera/camera_config.cpp
  src/camera/ids.cpp
  src/camera/distortion.cpp
  src/camera/pinhole_model.cpp
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
  src/feature2d/feature_container.cpp
  src/feature2d/feature_track.cpp
  src/feature2d/feature_tracker.cpp
  src/feature2d/gms_matcher.cpp
  src/feature2d/orb_tracker.cpp
  src/feature2d/klt_tracker.cpp
  # gimbal
  # src/gimbal/gimbal.cpp
  src/gimbal/sbgc.cpp
  # imu
  src/imu/imu.cpp
  src/imu/mpu6050.cpp
  # kitti
  src/kitti/raw/calib.cpp
  src/kitti/raw/oxts.cpp
  src/kitti/raw/parse.cpp
  src/kitti/raw/raw.cpp
  # pwm
  src/pwm/pca9685.cpp
  # quaternion
  src/quaternion/jpl.cpp
  # sim
  src/sim/carrot_controller.cpp
  src/sim/twowheel.cpp
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
SET(${PROJECT_NAME}_DEPS
  ${OpenCV_LIBS}
  ${SPQR_LIBRARIES}
  apriltags_mit
  apriltag
  yaml-cpp
  pthread
  ceres
  ueye_api
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/scripts DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_configs DESTINATION ${PROJECT_BINARY_DIR}/tests)
SET(UNITTESTS
  apriltag-mit_test
  camera-camera_test
  camera-camera_config_test
  camera-ids_test
  camera-pinhole_model_test
  feature2d-feature_container_test
  feature2d-feature_test
  feature2d-feature_track_test
  feature2d-feature_tracker_test
  feature2d-gms_matcher_test
  feature2d-klt_tracker_test
  feature2d-orb_tracker_test
  imu-mpu6050_test
  kitti-raw-calib_test
  kitti-raw-oxts_test
  kitti-raw-parse_test
  kitti-raw-raw_test
  msckf-camera_state_test
  msckf-feature_estimator_test
  msckf-imu_state_test
  msckf-msckf_test
  sim-carrot_controller_test
  sim-twowheel_test
  util-config_test
  util-data_test
  util-file_test
  util-gps_test
  util-linalg_test
  util-math_test
  util-stats_test
  util-time_test
)
FOREACH(TEST ${UNITTESTS})
  STRING(REGEX REPLACE "-" "/" TEST_PATH ${TEST})
  ADD_EXECUTABLE(${TEST} tests/${TEST_PATH}.cpp)
  TARGET_LINK_LIBRARIES(${TEST} ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
ENDFOREACH(TEST)
