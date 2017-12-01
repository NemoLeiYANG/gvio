# Download and unpack googletest at configure time
CONFIGURE_FILE(cmake/GetGtest.cmake.in googletest-download/CMakeLists.txt)
EXECUTE_PROCESS(
  COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download
)
IF(result)
  MESSAGE(FATAL_ERROR "CMake step for googletest failed: ${result}")
ENDIF()
EXECUTE_PROCESS(
  COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download
)
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
