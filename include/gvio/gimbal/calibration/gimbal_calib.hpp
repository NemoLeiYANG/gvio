/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_GIMBAL_CALIB_HPP
#define GVIO_GIMBAL_CALIBRATION_GIMBAL_CALIB_HPP

#include <ceres/ceres.h>

#include "gvio/util/util.hpp"
#include "gvio/gimbal/calibration/calib_data.hpp"
#include "gvio/gimbal/calibration/calib_params.hpp"
#include "gvio/gimbal/calibration/residual.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

class GimbalCalib {
public:
  CalibData data;
  CalibParams params;

  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  GimbalCalib();
  virtual ~GimbalCalib();

  /**
   * Load data
   *
   * @param data_dir Data directory
   * @return 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);

  /**
   * Calibrate gimbal
   */
  int calibrate();
};

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_GIMBAL_CALIB_HPP
