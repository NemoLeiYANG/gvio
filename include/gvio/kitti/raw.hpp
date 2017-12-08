#ifndef GVIO_KITTI_RAW_HPP
#define GVIO_KITTI_RAW_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "gvio/util/util.hpp"

namespace gvio {

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

/**
 * Camera to Camera calibration
 */
struct CalibCamToCam {
  bool ok = false;

  std::string calib_time;
  double corner_dist = 0.0;
  std::array<Vec2, 4> S;
  std::array<Mat3, 4> K;
  std::array<VecX, 4> D;
  std::array<Mat3, 4> R;
  std::array<Vec3, 4> T;
  std::array<Vec2, 4> S_rect;
  std::array<Mat3, 4> R_rect;
  std::array<Mat34, 4> P_rect;

  CalibCamToCam() {}

  /// Load calibration
  int load(const std::string &file_path);
};

/**
 * IMU to Camera calibration
 */
struct CalibIMUToCam {
  bool ok = false;

  std::string calib_time;
  Mat3 R;
  Vec3 t;

  CalibIMUToCam() {}

  /// Load calibration
  int load(const std::string &file_path);
};

/**
 * Velo to Camera calibration
 */
struct CalibVeloToCam {
  bool ok = false;

  std::string calib_time;
  Mat3 R;
  Vec3 t;
  Vec2 df;
  Vec2 dc;

  CalibVeloToCam() {}

  /// Load calibration
  int load(const std::string &file_path);
};

/**
 * Load KITTI Raw Dataset
 */
class RawDataset {
public:
  bool ok = false;

  std::string raw_dir;
  std::string date;
  std::string seq;

  CalibCamToCam calib_cam_to_cam;
  CalibIMUToCam calib_imu_to_cam;
  CalibVeloToCam calib_velo_to_cam;

  RawDataset(const std::string &raw_dir,
             const std::string &date,
             const std::string &seq)
      : raw_dir{strip_end(raw_dir, "/")}, date{date}, seq{seq} {}

  /// Load calibrations
  int loadCalibrations();

  /// Load raw dataset
  int load();
};

} // namespace gvio

#endif // GVIO_KITTI_RAW_HPP
