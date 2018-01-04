/**
 * @file
 * @ingroup kitti
 */
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
 * @addtogroup kitti
 * @{
 */

/**
 * OXTS entry
 */
struct OXTSEntry {
  Vec3 gps = zeros(3, 1);
  Vec3 rpy = zeros(3, 1);
  Vec3 v_G = zeros(3, 1);
  Vec3 v_B = zeros(3, 1);
  Vec3 a_G = zeros(3, 1);
  Vec3 a_B = zeros(3, 1);
  Vec3 w_G = zeros(3, 1);
  Vec3 w_B = zeros(3, 1);
  double pos_accuracy = 0.0;
  double vel_accuracy = 0.0;
};

/**
 * OXTS
 */
struct OXTS {
  bool ok = false;

  std::vector<double> timestamps;
  std::vector<Vec3> gps;
  std::vector<Vec3> rpy;
  std::vector<Vec3> p_G;
  std::vector<Vec3> v_G;
  std::vector<Vec3> v_B;
  std::vector<Vec3> a_G;
  std::vector<Vec3> a_B;
  std::vector<Vec3> w_G;
  std::vector<Vec3> w_B;
  std::vector<double> pos_accuracy;
  std::vector<double> vel_accuracy;

  OXTS() {}

  /**
   * Parse a single OXTS file
   *
   * @param file_path File path to a single OXTS data
   * @returns 0 for success, -1 for failure
   */
  int parseSingleOXTSFile(const std::string &file_path, OXTSEntry &entry);
  /**
   * Parse OXTS
   *
   * @param oxts_dir Path to OXTS data
   * @returns 0 for success, -1 for failure
   */
  int parseOXTS(const std::string &oxts_dir);

  /**
   * Parse a single timestamp to seconds since epoch (Unix time)
   *
   * @param line Timestamp line in the form of "2011-09-26 13:04:32.349659964"
   * @param s Seconds
   * @returns 0 for success, -1 for failure
   */
  int parseSingleTimeStamp(const std::string &line, double *s);

  /**
   * Parse timestamps
   *
   * @param oxts_dir Path to OXTS data
   * @returns 0 for success, -1 for failure
   */
  int parseTimeStamps(const std::string &oxts_dir);

  /**
   * Load OXTS
   *
   * @param oxts_dir Path to OXTS data
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &oxts_dir);
};

/** @} group kitti */
} // namespace gvio
#endif // GVIO_KITTI_RAW_OXTS_HPP
