/**
 * @file
 * @ingroup sim
 */
#ifndef GVIO_SIM_TWOWHEEL_HPP
#define GVIO_SIM_TWOWHEEL_HPP

#include "gvio/util/util.hpp"
#include "gvio/sim/pid.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Calculate target angular velocity and time taken to traverse a desired
 * circle * trajectory of radius r and velocity v
 *
 * @param r Desired circle radius
 * @param v Desired trajectory velocity
 * @param w Target angular velocity
 * @param time Target time taken to complete circle trajectory
 **/
void circle_trajectory(const double r, const double v, double *w, double *time);

/**
 * Two wheel robot
 */
class TwoWheelRobot {
public:
  Vec3 p_G = Vec3::Zero();
  Vec3 v_G = Vec3::Zero();
  Vec3 a_G = Vec3::Zero();
  Vec3 rpy_G = Vec3::Zero();
  Vec3 w_G = Vec3::Zero();

  double vx_desired = 0.0;
  double yaw_desired = 0.0;

  PID vx_controller{0.1, 0.0, 0.1};
  PID yaw_controller{0.1, 0.0, 0.1};

  Vec3 a_B = Vec3::Zero();
  Vec3 v_B = Vec3::Zero();
  Vec3 w_B = Vec3::Zero();

  TwoWheelRobot();
  TwoWheelRobot(const Vec3 &p_G, const Vec3 &v_G, const Vec3 &rpy_G);
  virtual ~TwoWheelRobot();

  /**
   * Update
   *
   * @param dt Time difference (s)
   */
  void update(const double dt);
};

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_TWOWHEEL_HPP
