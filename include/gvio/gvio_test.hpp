#ifndef GVIO_TEST_HPP
#define GVIO_TEST_HPP

#include <fstream>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

#ifdef TEST_OUTPUT_ON
#define TEST_PRINT(M, ...) fprintf(stdout, M "\n", ##__VA_ARGS__)
#endif

#ifndef TEST_DATA_PATH
#define TEST_DATA_PATH "test_data"
#endif

#ifndef TEST_KITTI_DATA
#define TEST_KITTI_DATA "test_data/kitti"
#endif

#endif // GVIO_TEST_HPP
