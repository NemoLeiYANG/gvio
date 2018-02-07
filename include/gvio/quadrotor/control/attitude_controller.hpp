/**
 * @file
 * @ingroup quadrotor
 */
#ifndef GVIO_QUADROTOR_CONTROL_ATTITUDE_CONTROLLER_HPP
#define GVIO_QUADROTOR_CONTROL_ATTITUDE_CONTROLLER_HPP

#include "gvio/util/util.hpp"
#include "gvio/control/pid.hpp"

namespace gvio {
/**
 * @addtogroup quadrotor
 * @{
 */

/**
 * Attitude controller
 */
class AttitudeController {
public:
  double dt = 0.0;
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  // PID tunes for dt = 0.001 (i.e. 1000Hz)
  PID roll_controller = PID(200.0, 0.5, 10.0);
  PID pitch_controller = PID(200.0, 0.5, 10.0);
  PID yaw_controller = PID(200.0, 0.5, 10.0);

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  // PID roll_controller = PID(10.0, 0.001, 1.0);
  // PID pitch_controller = PID(10.0, 0.001, 1.0);
  // PID yaw_controller = PID(10.0, 0.001, 1.0);

  AttitudeController() {}

  /**
   * Update
   *
   * @param setpoints Setpoints (roll, pitch, yaw, z)
   * @param actual Actual (roll, pitch, yaw, z)
   * @param dt Time difference (s)
   *
   * @returns Motor command (m1, m2, m3, m4)
   */
  Vec4 update(const Vec4 &setpoints, const Vec4 &actual, const double dt);
};

/** @} group quadrotor */
} // namespace gvio
#endif // GVIO_QUADROTOR_CONTROL_ATTITUDE_CONTROLLER_HPP
