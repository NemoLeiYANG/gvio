/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_RESIDUAL_HPP
#define GVIO_GIMBAL_CALIBRATION_RESIDUAL_HPP

#include <ceres/ceres.h>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

/**
 * Gimbal calibration residual
 */
struct GimbalCalibResidual {
  // Observed 3d point in static camera
  double P_s[3] = {0.0, 0.0, 0.0};

  // Observed 3d point in dynamic camera
  double P_d[3] = {0.0, 0.0, 0.0};

  // Observed pixel in static camera
  double Q_s[2] = {0.0, 0.0};

  // Observed pixel in dynamic camera
  double Q_d[2] = {0.0, 0.0};

  // Static camera intrinsics
  double fx_s = 0.0;
  double fy_s = 0.0;
  double cx_s = 0.0;
  double cy_s = 0.0;

  // Dynamic camera intrinsics
  double fx_d = 0.0;
  double fy_d = 0.0;
  double cx_d = 0.0;
  double cy_d = 0.0;

  GimbalCalibResidual();
  GimbalCalibResidual(const Vec3 &P_s,
                      const Vec3 &P_d,
                      const Vec2 &Q_s,
                      const Vec2 &Q_d,
                      const Mat3 &K_s,
                      const Mat3 &K_d);

  /// Form DH-Transform
  template <typename T>
  Eigen::Matrix<T, 4, 4>
  dhTransform(const T theta, const T alpha, const T a, const T d) const;

  /// Euler 3-2-1 to rotation matrix
  template <typename T>
  Eigen::Matrix<T, 3, 3> euler321ToRot(const T *euler) const;

  /// Form camera intrinsics matrix K
  template <typename T>
  Eigen::Matrix<T, 3, 3>
  K(const T fx, const T fy, const T cx, const T cy) const;

  /// Form static camera intrinsics matrix K_s
  template <typename T>
  Eigen::Matrix<T, 3, 3> K_s() const;

  /// Form dynamic camera intrinsics matrix K_d
  template <typename T>
  Eigen::Matrix<T, 3, 3> K_d() const;

  /// Form transform from static to dynamic camera
  template <typename T>
  Eigen::Matrix<T, 4, 4> T_sd(const T *const tau_s,
                              const T *const tau_d,
                              const T *const w1,
                              const T *const w2,
                              const T *const Lambda1,
                              const T *const Lambda2) const;

  /// Calculate residual
  template <typename T>
  bool operator()(const T *const tau_s,
                  const T *const tau_d,
                  const T *const w1,
                  const T *const w2,
                  const T *const Lambda1,
                  const T *const Lambda2,
                  T *residual) const;
};

/**
 * GimbalCalibResidual to string
 */
std::ostream &operator<<(std::ostream &os, const GimbalCalibResidual &residual);

/**
 * GimbalCalibResidual to string
 */
std::ostream &operator<<(std::ostream &os, const GimbalCalibResidual *residual);

/** @} group gimbal */
} // namespace gvio

#include "impl/residual.hpp"

#endif // GVIO_GIMBAL_CALIBRATION_RESIDUAL_HPP
