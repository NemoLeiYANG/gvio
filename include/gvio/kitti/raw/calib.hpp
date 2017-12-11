/**
 * @file
 * @ingroup kitti
 */
#ifndef GVIO_KITTI_RAW_CALIB_HPP
#define GVIO_KITTI_RAW_CALIB_HPP

#include "gvio/util/util.hpp"
#include "gvio/kitti/raw/calib.hpp"
#include "gvio/kitti/raw/parse.hpp"

namespace gvio {
/**
 * @addtogroup kitti
 * @{
 */

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

  /**
   * Load calibration
   */
  int load(const std::string &file_path);
};

/**
 * IMU to Velo calibration
 */
struct CalibIMUToVelo {
  bool ok = false;

  std::string calib_time;
  Mat3 R;
  Vec3 t;

  CalibIMUToVelo() {}

  /**
   * Load calibration
   */
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

  /**
   * Load calibration
   */
  int load(const std::string &file_path);
};

/** @} group kitti */
} // namespace gvio
#endif // GVIO_KITTI_RAW_CALIB_HPP
