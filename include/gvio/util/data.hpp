/**
 * @file
 * @defgroup data data
 * @ingroup util
 */
#ifndef GVIO_UTIL_DATA_HPP
#define GVIO_UTIL_DATA_HPP

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
#define E_CSV_DATA_LOAD "Error! failed to load data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"

/**
 * Get number of rows in CSV file
 *
 * @param file_path Path to CSV file
 * @returns Number of rows in CSV file
 */
int csvrows(const std::string &file_path);

/**
 * Get number of columns in CSV file
 *
 * @param file_path Path to CSV file
 * @returns Number of columns in CSV file
 */
int csvcols(const std::string &file_path);

/**
 * Convert CSV file to matrix
 *
 * @param file_path Path to CSV file
 * @param header Boolean to denote whether a header exists
 * @param data Matrix
 * @returns 0 for success, -1 for failure
 */
int csv2mat(const std::string &file_path, const bool header, MatX &data);

/**
 * Convert matrix to csv file
 *
 * @param file_path Path to CSV file
 * @param data Matrix
 * @returns 0 for success, -1 for failure
 */
int mat2csv(const std::string &file_path, const MatX &data);

/**
 * Print progress to screen
 *
 * @param percentage Percentage
 */
void print_progress(const double percentage);

/** @} group data */
} // namespace gvio
#endif // GVIO_UTIL_DATA_HPP
