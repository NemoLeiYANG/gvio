/**
 * @file
 * @ingroup kitti
 */
#ifndef GVIO_KITTI_RAW_HPP
#define GVIO_KITTI_RAW_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "gvio/util/util.hpp"
#include "gvio/kitti/raw/calib.hpp"
#include "gvio/kitti/raw/oxts.hpp"
#include "gvio/kitti/raw/parse.hpp"

namespace gvio {
/**
 * @addtogroup kitti
 * @{
 */

/**
 * Load KITTI Raw Dataset
 */
class RawDataset {
public:
  bool ok = false;

  std::string raw_dir;
  std::string date;
  std::string seq;
  std::string date_dir;
  std::string drive_dir;

  CalibCamToCam calib_cam_to_cam;
  CalibIMUToVelo calib_imu_to_velo;
  CalibVeloToCam calib_velo_to_cam;

  OXTS oxts;
  std::vector<std::string> cam0;
  std::vector<std::string> cam1;
  std::vector<std::string> cam2;
  std::vector<std::string> cam3;

  RawDataset(const std::string &raw_dir,
             const std::string &date,
             const std::string &seq)
      : raw_dir{strip_end(raw_dir, "/")}, date{date}, seq{seq},
        date_dir{raw_dir + "/" + date},
        drive_dir{date_dir + "/" + date + "_drive_" + seq + "_sync"} {}

  /// Load calibrations
  int loadCalibrations();

  /// Load image paths
  int loadImagePaths();

  /// Load OXTS
  int loadOXTS();

  /// Load raw dataset
  int load();
};

/** @} group kitti */
} // namespace gvio
#endif // GVIO_KITTI_RAW_HPP
