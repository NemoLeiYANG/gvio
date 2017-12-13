CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(gvio)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -fPIC -g")
IF (CMAKE_COMPILER_IS_GNUCXX)
  ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# DEPENDENCIES
FIND_PACKAGE(OpenCV 3.0 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE(cmake/GetGtest.cmake)
INCLUDE(cmake/ImportEigen3.cmake)

# INCLUDES
INCLUDE_DIRECTORIES(include /usr/include/eigen3)

# LIBRARY
ADD_LIBRARY(
	${PROJECT_NAME}
	# apriltag
	src/apriltag/base_detector.cpp
	src/apriltag/mit.cpp
	src/apriltag/michigan.cpp
	# camera
	src/camera/camera.cpp
	src/camera/camera_config.cpp
	# driver
	src/driver/i2c.cpp
	src/driver/uart.cpp
  # msckf
  src/msckf/camera_state.cpp
  src/msckf/imu_state.cpp
  # feature
	src/feature2d/feature_tracker.cpp
	src/feature2d/gms_matcher.cpp
	src/feature2d/orb_tracker.cpp
	src/feature2d/klt_tracker.cpp
	# gimbal
	# src/gimbal/gimbal.cpp
	src/gimbal/sbgc.cpp
	# imu
	src/imu/mpu6050.cpp
  # kitti
	src/kitti/raw/calib.cpp
	src/kitti/raw/oxts.cpp
	src/kitti/raw/parse.cpp
	src/kitti/raw/raw.cpp
	# quaternion
	src/quaternion/jpl.cpp
	# utils
	src/util/config.cpp
	src/util/data.cpp
	src/util/file.cpp
	src/util/gps.cpp
	src/util/linalg.cpp
	src/util/math.cpp
	src/util/opencv.cpp
	src/util/stats.cpp
	src/util/time.cpp
)
SET(${PROJECT_NAME}_DEPS
	${OpenCV_LIBS}
	apriltags_mit
	apriltag
	yaml-cpp
	pthread
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})

# UNIT TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_data DESTINATION ${PROJECT_BINARY_DIR}/tests)
FILE(COPY tests/test_configs DESTINATION ${PROJECT_BINARY_DIR}/tests)
ADD_EXECUTABLE(
	gvio_tests
	# imu
	tests/imu/mpu6050_test.cpp
	# apriltag
	# tests/apriltag/mit_test.cpp
	# camera
	tests/camera/camera_config_test.cpp
	tests/camera/camera_test.cpp
  # feature2d
  tests/feature2d/feature_tracker_test.cpp
	tests/feature2d/gms_matcher_test.cpp
  tests/feature2d/orb_tracker_test.cpp
  tests/feature2d/klt_tracker_test.cpp
	# gimbal
	# tests/gimbal/gimbal_test.cpp
	tests/gimbal/sbgc_test.cpp
  # kitti
	tests/kitti/raw/calib_test.cpp
	tests/kitti/raw/oxts_test.cpp
	tests/kitti/raw/parse_test.cpp
	tests/kitti/raw/raw_test.cpp
	# util
	tests/util/config_test.cpp
	tests/util/data_test.cpp
	tests/util/file_test.cpp
	tests/util/gps_test.cpp
	tests/util/math_test.cpp
	tests/util/stats_test.cpp
	tests/util/time_test.cpp
	# test runner
	tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(gvio_tests ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS} gtest)
