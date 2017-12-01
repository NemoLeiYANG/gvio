CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(gvio)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC -g")
IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

FIND_PACKAGE(OpenCV 3.0 REQUIRED)

# Download and unpack googletest at configure time
CONFIGURE_FILE(CMakeLists.txt.in googletest-download/CMakeLists.txt)
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
IF(result)
  MESSAGE(FATAL_ERROR "CMake step for googletest failed: ${result}")
ENDIF()
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
IF(result)
  MESSAGE(FATAL_ERROR "Build step for googletest failed: ${result}")
ENDIF()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
SET(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
ADD_SUBDIRECTORY(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
IF(CMAKE_VERSION VERSION_LESS 2.8.11)
  INCLUDE_DIRECTORIES("${gtest_SOURCE_DIR}/include")
ENDIF()


# INCLUDES
INCLUDE_DIRECTORIES(include
										/usr/include/eigen3)

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
