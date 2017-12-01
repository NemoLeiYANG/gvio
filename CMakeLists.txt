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
	STATIC
	# apriltag
	src/apriltag/base_detector.cpp
	src/apriltag/mit.cpp
	# camera
	src/camera/camera.cpp
	src/camera/config.cpp
	# driver
	src/driver/i2c.cpp
	src/driver/uart.cpp
  # feature
	src/feature/feature_tracker.cpp
	# gimbal
	# src/gimbal/gimbal.cpp
	src/gimbal/sbgc.cpp
	# imu
	src/imu/mpu6050.cpp
	# utils
	src/util/config.cpp
	src/util/data.cpp
	src/util/file.cpp
	src/util/gps.cpp
	src/util/math.cpp
	src/util/opencv.cpp
	src/util/stats.cpp
	src/util/time.cpp
)
SET(${PROJECT_NAME}_DEPS
	${OpenCV_LIBS}
	apriltags_mit
	yaml-cpp
	gtest
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
	tests/camera/camera_test.cpp
	tests/camera/config_test.cpp
	# gimbal
	# tests/gimbal/gimbal_test.cpp
	tests/gimbal/sbgc_test.cpp
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
TARGET_LINK_LIBRARIES(gvio_tests ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})
