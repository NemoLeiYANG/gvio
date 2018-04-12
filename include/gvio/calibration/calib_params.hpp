/**
 * @file
 * @ingroup calibration
 */
#ifndef GVIO_CALIBRATION_CALIB_PARAMS_HPP
#define GVIO_CALIBRATION_CALIB_PARAMS_HPP

#include "gvio/util/util.hpp"
#include "gvio/calibration/camchain.hpp"

namespace gvio {
/**
 * @addtogroup calibration
 * @{
 */

/**
 * Calibration parameters
 */
struct CalibParams {
  Camchain camchain;

  double *tau_s = nullptr;
  double *tau_d = nullptr;
  double *w1 = nullptr;
  double *w2 = nullptr;
  double *theta1_offset = nullptr;
  double *theta2_offset = nullptr;
  double *Lambda1 = nullptr;
  double *Lambda2 = nullptr;
  int nb_measurements = 0;

  CalibParams();
  virtual ~CalibParams();

  /**
   * Load initial optimization params
   *
   * @param camchain_file Path to camchain file
   * @param joint_file Path to joint angles file
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &camchain_file, const std::string &joint_file);
};

/**
 * CalibParams to string
 */
std::ostream &operator<<(std::ostream &os, const CalibParams &m);

/** @} group calibration */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CALIB_PARAMS_HPP
