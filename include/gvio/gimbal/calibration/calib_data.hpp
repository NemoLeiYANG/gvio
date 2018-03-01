/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CALIBDATA_HPP
#define GVIO_GIMBAL_CALIBRATION_CALIBDATA_HPP

#include <string>
#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

struct CalibData {
  int nb_measurements = 0;
  std::vector<MatX> P_s;
  std::vector<MatX> P_d;
  std::vector<MatX> Q_s;
  std::vector<MatX> Q_d;
  MatX joint_data;

  /**
   * Load data
   *
   * @param data_dir Data directory
   * @return 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);
};

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CALIB_DATA_HPP
