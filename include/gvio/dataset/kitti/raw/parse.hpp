/**
 * @file
 * @defgroup kitti kitti
 * @ingroup dataset
 */
#ifndef GVIO_DATASET_KITTI_RAW_PARSE_HPP
#define GVIO_DATASET_KITTI_RAW_PARSE_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup kitti
 * @{
 */

/// Parse calibration string
std::string parseString(const std::string &line);

/// Parse double
double parseDouble(const std::string &line);

/// Parse array
std::vector<double> parseArray(const std::string &line);

/// Parse vector of size 2
Vec2 parseVec2(const std::string &line);

/// Parse vector of size 3
Vec3 parseVec3(const std::string &line);

/// Parse vector
VecX parseVecX(const std::string &line);

/// Parse 3x3 matrix
Mat3 parseMat3(const std::string &line);

/// Parse 3x4 matrix
Mat34 parseMat34(const std::string &line);

/** @} group kitti */
} // namespace gvio
#endif // GVIO_DATASET_KITTI_RAW_PARSE_HPP
