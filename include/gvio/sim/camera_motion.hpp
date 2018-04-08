/**
 * @file
 * @defgroup sim sim
 */
#ifndef GVIO_SIM_MOTION_HPP
#define GVIO_SIM_MOTION_HPP

#include <vector>

#include "gvio/sim/bezier.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

class CameraMotion {
public:
  // Simulation settings
  std::vector<Vec3> control_points; ///< Bezier curve control points
  double max_time = 0.0;            ///< Simulation max time [s]

  // Camera position velocity and accelation
  Vec3 p_G = Vec3::Zero();   ///< Global position [m]
  Vec3 v_G = Vec3::Zero();   ///< Global velocity [m s^-1]
  Vec3 a_G = Vec3::Zero();   ///< Global acceleration [m s^-2]
  Vec3 rpy_G = Vec3::Zero(); ///< Global attitude (roll, pitch, yaw) [rad]
  Vec3 w_G = Vec3::Zero();   ///< Global angular velocity [rad]

  // Emulate IMU measurements
  Vec3 a_B = Vec3::Zero(); ///< Body acceleration [m s^-2]
  Vec3 w_B = Vec3::Zero(); ///< Body angular velocity [rad s^-1]

  CameraMotion();

  CameraMotion(const std::vector<Vec3> &control_points, const double max_time);

  virtual ~CameraMotion();

  /**
   * Update
   *
   * @time Current simulation time [s]
   */
  void update(const double time);
};

/**
 * CameraMotion to string
 */
std::ostream &operator<<(std::ostream &os, const CameraMotion &camera_motion);

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_CAMERA_MOTION_HPP
