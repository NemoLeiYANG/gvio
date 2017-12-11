/**
 * @file
 * @ingroup msckf
 */
#ifndef GVIO_MSCKF_IMU_STATE_HPP
#define GVIO_MSCKF_IMU_STATE_HPP

#include "gvio/util/util.hpp"
#include "gvio/quaternion/jpl.hpp"

namespace gvio {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * IMU State
 */
struct IMUState {
  const double size = 15; ///< Size of state vector

  Vec4 q_IG; ///< JPL Quaternion of IMU in Global frame
  Vec3 b_g;  ///< Bias of gyroscope
  Vec3 v_G;  ///< Velocity of IMU in Global frame
  Vec3 b_a;  ///< Bias of accelerometer
  Vec3 p_G;  ///< Position of IMU in Global frame

  Vec3 w_G; ///< Gravitational angular velocity
  Vec3 g_G; ///< Gravitational acceleration

  MatX P; ///< Covariance matrix
  MatX Q; ///< Noise matrix

  MatX Phi; ///< Phi matrix

  IMUState() {}

  /**
   * Transition Jacobian F matrix
   *
   * @param w_hat Estimated angular velocity
   * @param q_hat Estimated quaternion (x, y, z, w)
   * @param a_hat Estimated acceleration
   * @param w_G Earth's angular velocity (i.e. Earth's rotation)
   * @returns Transition jacobian matrix F
   */
  MatX
  F(const Vec3 &w_hat, const Vec4 &q_hat, const Vec3 &a_hat, const Vec3 &w_G);

  /**
   * Input Jacobian G matrix
   *
   * A matrix that maps the input vector (IMU gaussian noise) to the state
   * vector (IMU error state vector), it tells us how the inputs affect the
   * state vector.
   *
   * @param q_hat Estimated quaternion (x, y, z, w)
   * @returns Input jacobian matrix G
   */
  MatX G(const Vec4 &q_hat);

  /**
   * Jacobian J matrix
   *
   * @param cam_q_CI Rotation from IMU to camera frame in quaternion (x, y, z,
   * w)
   * @param cam_p_IC Position of camera in IMU frame
   * @param q_hat_IG Rotation from global to IMU frame
   * @param N Number of camera states
   */
  MatX J(const Vec4 &cam_q_CI,
         const Vec3 &cam_p_IC,
         const Vec4 &q_hat_IG,
         const double N);

  /**
   * Update
   *
   * @param a_m Measured acceleration
   * @param w_m Measured angular velocity
   * @param dt Time difference in seconds
   */
  void update(const Vec3 &a_m, const Vec3 &w_m, const double dt);

  /**
   * Correct the IMU state
   *
   * @param dx Correction state vector
   */
  void correct(const VecX &dx);
};

/** @} group msckf */
} // namespace gvio
#endif // GVIO_MSCKF_IMU_STATE_HPP
