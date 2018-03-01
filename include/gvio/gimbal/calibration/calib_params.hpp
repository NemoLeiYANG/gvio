/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CALIB_PARAMS_HPP
#define GVIO_GIMBAL_CALIBRATION_CALIB_PARAMS_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

/**
 * Calibration parameters
 */
struct CalibParams {
  double *tau_s = nullptr;
  double *tau_d = nullptr;
  double *w1 = nullptr;
  double *w2 = nullptr;
  double *Lambda1 = nullptr;
  double *Lambda2 = nullptr;
  int nb_measurements = 0;

  CalibParams();
  virtual ~CalibParams();

  /**
   * Load initial optimization params
   *
   * @param config_file Path to config file
   * @param joint_file Path to joint angles file
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &config_file, const std::string &joint_file);
};

/**
 * CalibParams to string
 */
std::ostream &operator<<(std::ostream &os, const CalibParams &m);

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CALIB_PARAMS_HPP
