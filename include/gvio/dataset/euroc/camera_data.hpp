/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef GVIO_DATASET_EUROC_CAMERA_DATA_HPP
#define GVIO_DATASET_EUROC_CAMERA_DATA_HPP

#include <iostream>
#include <vector>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup euroc
 * @{
 */

/**
 * Camera data
 */
struct CameraData {
  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<std::string> image_paths;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  Mat4 T_BS = I(4);
  double rate_hz = 0.0;
  Vec2 resolution;
  std::string camera_model;
  Vec4 intrinsics;
  std::string distortion_model;
  Vec4 distortion_coefficients;

  /**
   * Load Camera data
   *
   * @param data_dir Camera data directory
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);
};

/**
 * CameraData to output stream
 */
std::ostream &operator<<(std::ostream &os, const CameraData &data);

/** @} group euroc */
} // namespace gvio
#endif // GVIO_DATASET_EUROC_CAMERA_DATA_HPP
