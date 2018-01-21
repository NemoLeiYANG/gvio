/**
 * @file
 * @ingroup sim
 */
#ifndef GVIO_SIM_PID_HPP
#define GVIO_SIM_PID_HPP

#include <float.h>
#include <iostream>
#include <math.h>

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

/**
 * PID Controller
 */
class PID {
public:
  double error_prev = 0.0;
  double error_sum = 0.0;

  double error_p = 0.0;
  double error_i = 0.0;
  double error_d = 0.0;

  double k_p = 0.0;
  double k_i = 0.0;
  double k_d = 0.0;

  PID() {}
  PID(const double k_p, const double k_i, const double k_d)
      : k_p(k_p), k_i(k_i), k_d(k_d) {}

  /**
   * Update controller
   *
   * @param setpoint Setpoint
   * @param actual Actual
   * @param dt Time step
   * @returns Controller command
   */
  double update(const double setpoint, const double actual, const double dt);

  /**
   * Update controller
   *
   * @param error
   * @param dt Difference in time
   * @returns Controller command
   */
  double update(const double error, const double dt);

  /**
   * Reset controller
   */
  void reset();
};

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_PID_HPP
