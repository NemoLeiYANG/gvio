/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_GIMBAL_CALIB_HPP
#define GVIO_GIMBAL_CALIBRATION_GIMBAL_CALIB_HPP

#include <string>

#include <ceres/ceres.h>

#include "gvio/util/util.hpp"
#include "gvio/gimbal/calibration/calib_data.hpp"
#include "gvio/gimbal/calibration/residual.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

class GimbalCalib {
public:
  CalibData data;
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  double *tau_s;
  double *Lambda1;
  double *w1;
  double *Lambda2;
  double *w2;
  double *tau_d;

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
