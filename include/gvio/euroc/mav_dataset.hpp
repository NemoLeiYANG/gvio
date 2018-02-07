/**
 * @file
 * @ingroup euroc
 */
#ifndef GVIO_EUROC_MAV_DATASET_HPP
#define GVIO_EUROC_MAV_DATASET_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

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
  std::vector<double> timestamps;
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

struct CameraData {
  // Data
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

/**
 * EuRoC MAV Dataset
 */
class MAVDataset {
public:
  bool ok = false;

  std::string data_path;

  // IMU data
  IMUData imu_data;

  // Camera data
  CameraData cam0_data;
  CameraData cam1_data;

  MAVDataset(const std::string &data_path)
      : data_path{strip_end(data_path, "/")} {}

  /// Load imu data
  int loadIMUData();

  /// Load camera data
  int loadCameraData();

  /// Load
  int load();
};

/** @} group euroc */
} // namespace gvio
#endif // GVIO_EUROC_MAV_DATASET_HPP
