/**
 * @file
 * @defgroup data data
 * @ingroup util
 */
#ifndef GVIO_UTILS_DATA_HPP
#define GVIO_UTILS_DATA_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "gvio/util/math.hpp"

namespace gvio {
/**
 * @addtogroup data
 * @{
 */

// CSV ERROR MESSAGES
#define E_CSV_DATA_LOAD "Error! failed to load test data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"

int csvrows(const std::string &file_path);
int csvcols(const std::string &file_path);
int csv2mat(const std::string &file_path, const bool header, MatX &data);
int mat2csv(const std::string &file_path, MatX data);

/** @} group data */
} // namespace gvio
#endif
