#ifndef GVIO_KITTI_RAW_RAW_HPP
#define GVIO_KITTI_RAW_RAW_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "gvio/util/util.hpp"
#include "gvio/kitti/raw/parse.hpp"
#include "gvio/kitti/raw/calib.hpp"

namespace gvio {

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
#endif // GVIO_KITTI_RAW_RAW_HPP
