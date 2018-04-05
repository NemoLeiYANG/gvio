/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef GVIO_DATASET_EUROC_IMU_DATA_HPP
#define GVIO_DATASET_EUROC_IMU_DATA_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup euroc
 * @{
 */

/**
 * IMU data
 */
struct IMUData {
  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<Vec3> w_B;
  std::vector<Vec3> a_B;

  // Sensor properties
  std::string sensor_type;
  std::string comment;
  Mat4 T_BS = I(4);
  double rate_hz = 0.0;
  double gyro_noise_density = 0.0;
  double gyro_random_walk = 0.0;
  double accel_noise_density = 0.0;
  double accel_random_walk = 0.0;

  /**
   * Load IMU data
   *
   * @param data_dir IMU data directory
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);
};

/**
 * IMUData to output stream
 */
std::ostream &operator<<(std::ostream &os, const IMUData &data);

/** @} group euroc */
} // namespace gvio
#endif // GVIO_DATASET_EUROC_IMU_DATA_HPP
