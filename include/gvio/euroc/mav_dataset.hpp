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

struct GroundTruthData {
  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<Vec3> p_RS_R;
  std::vector<Vec4> q_RS;
  std::vector<Vec3> v_RS_R;
  std::vector<Vec3> b_w_RS_S;
  std::vector<Vec3> b_a_RS_S;

  /**
   * Load ground truth data
   *
   * @param data_dir Ground truth data directory
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);
};

/**
 * EuRoC MAV Dataset
 */
class MAVDataset {
public:
  bool ok = false;
  std::string data_path;

  IMUData imu_data;
  CameraData cam0_data;
  CameraData cam1_data;
  GroundTruthData ground_truth;

  long ts_start = 0;
  long ts_end = 0;

  MAVDataset(const std::string &data_path)
      : data_path{strip_end(data_path, "/")} {}

  /**
   * Load imu data
   * @returns 0 for success, -1 for failure
   */
  int loadIMUData();

  /**
   * Load camera data
   * @returns 0 for success, -1 for failure
   */
  int loadCameraData();

  /**
   * Load ground truth data
   * @returns 0 for success, -1 for failure
   */
  int loadGroundTruthData();

  /**
   * Return min timestamp
   */
  long minTimestamp();

  /**
   * Return max timestamp
   */
  long maxTimestamp();

  /**
   * Load data
   * @returns 0 for success, -1 for failure
   */
  int load();
};

/** @} group euroc */
} // namespace gvio
#endif // GVIO_EUROC_MAV_DATASET_HPP
