/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef GVIO_DATASET_EUROC_MAV_DATASET_HPP
#define GVIO_DATASET_EUROC_MAV_DATASET_HPP

#include <iostream>
#include <functional>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "gvio/util/util.hpp"
#include "gvio/msckf/msckf.hpp"

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

enum DatasetEventType { NOT_SET, IMU_EVENT, CAMERA_EVENT };

struct DatasetEvent {
  // Event Type
  DatasetEventType type;

  // IMU data
  Vec3 a_m;
  Vec3 w_m;

  // Camera data
  int camera_index;
  std::string image_path;

  DatasetEvent(const Vec3 &a_m, const Vec3 &w_m)
      : type{IMU_EVENT}, a_m{a_m}, w_m{w_m} {}
  DatasetEvent(const int camera_index, const std::string &image_path)
      : type{CAMERA_EVENT}, camera_index{camera_index}, image_path{image_path} {
  }
};

/**
 * DatasetEvent to output stream
 */
std::ostream &operator<<(std::ostream &os, const DatasetEvent &data);

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
  long ts_now = 0;
  long time_index = 0;
  long imu_index = 0;
  long frame_index = 0;

  std::vector<long> timestamps;
  std::map<long, double> time;
  std::multimap<long, DatasetEvent> timeline;

  // clang-format off
  std::function<VecX()> get_state;
  std::function<int(const Vec3 &a_m, const Vec3 &w_m, const long ts)> imu_cb;
  std::function<int(const cv::Mat &frame, const long ts)> mono_camera_cb;
  std::function<int(const cv::Mat &frame0, const cv::Mat &frame1, const long ts)> stereo_camera_cb;
  std::function<int(const double time, const Vec3 &p_G, const Vec3 &v_G, const Vec3 &rpy_G)> record_cb;
  // clang-format on

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

  /**
   * Reset
   */
  void reset();

  /**
   * Step
   *
   * @returns
   * - 0 for success
   * - -2 for imu callback failure
   * - -3 for camera callback failure
   */
  int step();

  /**
   * Run
   *
   * @returns
   * - 0 for success
   * - -2 for imu callback failure
   * - -3 for camera callback failure
   */
  int run();
};

/** @} group euroc */
} // namespace gvio
#endif // GVIO_DATASET_EUROC_MAV_DATASET_HPP
