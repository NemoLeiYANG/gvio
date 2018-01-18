/**
 * @file
 * @ingroup sim
 */
#ifndef GVIO_SIM_TWOWHEEL_HPP
#define GVIO_SIM_TWOWHEEL_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Calculate target angular velocity given a desired circle
 * trajectory of radius r and velocity v
 *
 * @param r Desired circle radius
 * @param v Desired trajectory velocity
 * @param w Target angular velocity to complete a circle of radius r and
 *          velocity v
 * @param time Target time taken to complete circle trajectory
 **/
void circle_trajectory(const double r, const double v, double *w, double *time);

/**
 * Two wheel robot
 */
class TwoWheelRobot {
public:
  Vec3 p_G = Vec3::Zero();
  Vec3 rpy_G = Vec3::Zero();

  Vec3 w_B = Vec3::Zero();
  Vec3 v_B = Vec3::Zero();
  Vec3 a_B = Vec3::Zero();

  TwoWheelRobot() {}
  virtual ~TwoWheelRobot() {}

  /**
   * Update
   *
   * @param ax_B Body forward x acceleration (m/s)
   * @param wz_B Body angular z velocity (m/s)
   * @param dt Time difference (s)
   */
  void update(const double ax_B, const double wz_B, const double dt);
};

} // namespace gvio
#endif // GVIO_SIM_TWOWHEEL_HPP
