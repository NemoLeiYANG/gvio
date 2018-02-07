/**
 * @file
 * @ingroup quadrotor
 */
#ifndef GVIO_QUADROTOR_CONTROL_POSITION_CONTROLLER_HPP
#define GVIO_QUADROTOR_CONTROL_POSITION_CONTROLLER_HPP

#include "gvio/util/util.hpp"
#include "gvio/control/pid.hpp"

namespace gvio {
/**
 * @addtogroup quadrotor
 * @{
 */

/**
 * Position controller
 */
class PositionController {
public:
  double dt = 0.0;
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  // PID tunes for dt = 0.001 (i.e. 1000Hz)
  PID x_controller = PID(0.5, 0.0, 0.035);
  PID y_controller = PID(0.5, 0.0, 0.035);
  PID z_controller = PID(0.3, 0.0, 0.1);

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  // PID x_controller = PID(5.0, 0.0, 3.5);
  // PID y_controller = PID(5.0, 0.0, 3.5);
  // PID z_controller = PID(3.0, 0.0, 1.0);

  PositionController() {}

  /**
   * Update
   *
   * @param setpoints Setpoints (x, y, z)
   * @param actual Actual (x, y, z)
   * @param yaw (radians)
   * @param dt Time difference (s)
   *
   * @returns Attitude command (roll, pitch, yaw, thrust)
   */
  Vec4 update(const Vec3 &setpoints,
              const Vec4 &actual,
              const double yaw,
              const double dt);

  /**
   * Reset controller
   */
  void reset();

  /**
   * Print outputs
   */
  void printOutputs();
};

/** @} group quadrotor */
} // namespace gvio
#endif // GVIO_QUADROTOR_CONTROL_POSITION_CONTROLLER_HPP
