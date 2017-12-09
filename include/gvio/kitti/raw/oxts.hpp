#ifndef GVIO_KITTI_RAW_OXTS_HPP
#define GVIO_KITTI_RAW_OXTS_HPP

#include <algorithm>
#include <vector>
#include <string>
#include <chrono>

#include "gvio/util/util.hpp"
#include "gvio/kitti/raw/parse.hpp"

namespace gvio {

/**
 * OXTS
 */
struct OXTS {
  bool ok = false;

  std::vector<double> timestamps;
  std::vector<Vec3> gps;
  std::vector<Vec3> rpy;
  std::vector<Vec3> v_G;
  std::vector<Vec3> v_B;
  std::vector<Vec3> a_G;
  std::vector<Vec3> a_B;
  std::vector<Vec3> w_G;
  std::vector<Vec3> w_B;
  std::vector<double> pos_accuracy;
  std::vector<double> vel_accuracy;

  OXTS() {}

  /// Parse OXTS
  int parseOXTS(const std::string &oxts_dir);

  /// Parse a single timestamp
  int parseSingleTimeStamp(const std::string &line, double *s);

  /// Parse timestamps
  int parseTimeStamps(const std::string &oxts_dir);

  /// Load OXTS
  int load(const std::string &oxts_dir);
};

} // namespace gvio
#endif // GVIO_KITTI_RAW_OXTS_HPP
