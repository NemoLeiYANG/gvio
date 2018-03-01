/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_GIMBAL_MODEL_HPP
#define GVIO_GIMBAL_GIMBAL_MODEL_HPP

#include <string>

#include <apriltags_mit/TagDetector.h>
#include <apriltags_mit/Tag36h11.h>

#include <ceres/ceres.h>
#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

/**
 * Create DH transform
 *
 * @param theta
 * @param alpha
 * @param a
 * @param d
 *
 * @returns DH transform
 */
Mat4 dh_transform(const double theta,
                  const double alpha,
                  const double a,
                  const double d);

class GimbalModel {
public:
  VecX tau_s = zeros(6, 1);
  double Lambda1 = 0.0;
  Vec3 w1 = zeros(3, 1);
  double Lambda2 = 0.0;
  Vec3 w2 = zeros(3, 1);
  VecX tau_d = zeros(6, 1);

  Vec2 attitude;
  double width;
  double length;

  GimbalModel();
  virtual ~GimbalModel();

  /**
   * Set gimbal attitude
   *
   * @param roll Roll (radians)
   * @param pitch Pitch (radians)
   */
  void setAttitude(const double roll, const double pitch);

  /**
   * Returns transform from static camera to base mechanism
   */
  Mat4 T_sb();

  /**
   * Returns transform from base mechanism to end-effector
   */
  Mat4 T_be();

  /**
   * Returns transform from end-effector to dynamic camera
   */
  Mat4 T_ed();

  /**
   * Returns transform from static to dynamic camera
   */
  Mat4 T_sd();
};

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_GIMBAL_MODEL_HPP
