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
 * Create DH transform from link n to link n-1 (end to front)
 *
 * @param theta
 * @param d
 * @param a
 * @param alpha
 *
 * @returns DH transform
 */
Mat4 dh_transform(const double theta,
                  const double d,
                  const double a,
                  const double alpha);

class GimbalModel {
public:
  VecX tau_s = zeros(6, 1);
  double Lambda1 = 0.0;
  Vec3 w1 = zeros(3, 1);
  double Lambda2 = 0.0;
  Vec3 w2 = zeros(3, 1);
  VecX tau_d = zeros(6, 1);

  double theta1_offset = 0.0;
  double theta2_offset = 0.0;

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
  Mat4 T_bs();

  /**
   * Returns transform from base mechanism to end-effector
   */
  Mat4 T_eb();

  /**
   * Returns transform from end-effector to dynamic camera
   */
  Mat4 T_de();

  /**
   * Returns transform from static to dynamic camera
   */
  Mat4 T_ds();
};

std::ostream &operator<<(std::ostream &os, const GimbalModel &gimbal);

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_GIMBAL_MODEL_HPP
