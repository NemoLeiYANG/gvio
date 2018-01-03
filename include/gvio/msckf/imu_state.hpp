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
class IMUState {
public:
  static const int size = 15; ///< Size of state vector

  Vec4 q_IG = zeros(4, 1); ///< JPL Quaternion of IMU in Global frame
  Vec3 b_g = zeros(3, 1);  ///< Bias of gyroscope
  Vec3 v_G = zeros(3, 1);  ///< Velocity of IMU in Global frame
  Vec3 b_a = zeros(3, 1);  ///< Bias of accelerometer
  Vec3 p_G = zeros(3, 1);  ///< Position of IMU in Global frame

  Vec3 w_G = zeros(3, 1); ///< Earth's angular velocity
  Vec3 g_G = zeros(3, 1); ///< Gravitational acceleration

  MatX P = zeros(15);   ///< Covariance matrix
  MatX Q = zeros(12);   ///< Noise matrix
  MatX Phi = zeros(15); ///< Phi matrix

  IMUState() {
  }

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
         const int N);

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
