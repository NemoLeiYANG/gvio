/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_IMPL_RESIDUAL_HPP
#define GVIO_GIMBAL_CALIBRATION_IMPL_RESIDUAL_HPP

#include <ceres/ceres.h>

#include "gvio/util/util.hpp"
#include "gvio/gimbal/calibration/gimbal_calib.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

template <typename T>
Eigen::Matrix<T, 4, 4> GimbalCalibResidual::dhTransform(const T theta,
                                                        const T alpha,
                                                        const T a,
                                                        const T d) const {
  // clang-format off
  Eigen::Matrix<T, 4, 4> transform;
  transform <<
    cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
    sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
    0.0, sin(alpha), cos(alpha), d,
    0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return transform;
}

template <typename T>
Eigen::Matrix<T, 3, 3>
GimbalCalibResidual::euler321ToRot(const T *const euler) const {
  // i.e. ZYX rotation sequence (world to body)
  const T phi = euler[0];
  const T theta = euler[1];
  const T psi = euler[2];

  const T R11 = cos(psi) * cos(theta);
  const T R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const T R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);

  const T R21 = sin(psi) * cos(theta);
  const T R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const T R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);

  const T R31 = -sin(theta);
  const T R32 = cos(theta) * sin(phi);
  const T R33 = cos(theta) * cos(phi);

  Eigen::Matrix<T, 3, 3> R;
  // clang-format off
  R << R11, R12, R13,
        R21, R22, R23,
        R31, R32, R33;
  // clang-format on

  return R;
}

template <typename T>
Eigen::Matrix<T, 3, 3>
GimbalCalibResidual::K(const T fx, const T fy, const T cx, const T cy) const {
  // clang-format off
  Eigen::Matrix<T, 3, 3> K;
  K << fx, 0.0, cx,
       0.0, fy, cy,
       0.0, 0.0, 1.0;
  return K;
  // clang-format on
}

template <typename T>
Eigen::Matrix<T, 4, 4> GimbalCalibResidual::T_sd(const T *const tau_s,
                                                 const T *const tau_d,
                                                 const T *const w1,
                                                 const T *const w2,
                                                 const T *const Lambda1,
                                                 const T *const Lambda2) const {
  // Form T_sb
  const Eigen::Matrix<T, 3, 1> t_G_sb{tau_s[0], tau_s[1], tau_s[2]};
  const T rpy_bs[3] = {tau_s[3], tau_s[4], tau_s[5]};
  const Eigen::Matrix<T, 3, 3> R_sb = this->euler321ToRot(rpy_bs);
  // clang-format off
  Eigen::Matrix<T, 4, 4> T_sb;
  T_sb << R_sb(0, 0), R_sb(0, 1), R_sb(0, 2), t_G_sb(0),
          R_sb(1, 0), R_sb(1, 1), R_sb(1, 2), t_G_sb(1),
          R_sb(2, 0), R_sb(2, 1), R_sb(2, 2), t_G_sb(2),
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Form T_eb
  // -- DH params for first link
  const T theta1 = Lambda1[0];
  const T alpha1 = w1[0];
  const T a1 = w1[1];
  const T d1 = w1[2];
  // -- DH params for second link
  const T theta2 = Lambda2[0];
  const T alpha2 = w2[0];
  const T a2 = w2[1];
  const T d2 = w2[2];
  // -- Combine DH transforms to form T_eb
  // clang-format off
  const Eigen::Matrix<T, 4, 4> T_b1 = this->dhTransform(theta1, alpha1, a1, d1);
  const Eigen::Matrix<T, 4, 4> T_1e = this->dhTransform(theta2, alpha2, a2, d2);
  const Eigen::Matrix<T, 4, 4> T_be = T_b1 * T_1e;
  // clang-format on

  // Form T_ed
  const Eigen::Matrix<T, 3, 1> t_d_ed{tau_d[0], tau_d[1], tau_d[2]};
  const T rpy_de[3] = {tau_d[3], tau_d[4], tau_d[5]};
  const Eigen::Matrix<T, 3, 3> R_ed = this->euler321ToRot(rpy_de);
  // clang-format off
  Eigen::Matrix<T, 4, 4> T_ed;
  T_ed << R_ed(0, 0), R_ed(0, 1), R_ed(0, 2), t_d_ed(0),
          R_ed(1, 0), R_ed(1, 1), R_ed(1, 2), t_d_ed(1),
          R_ed(2, 0), R_ed(2, 1), R_ed(2, 2), t_d_ed(2),
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T_sb * T_be * T_ed;
}

template <typename T>
bool GimbalCalibResidual::operator()(const T *const tau_s,
                                     const T *const tau_d,
                                     const T *const w1,
                                     const T *const w2,
                                     const T *const Lambda1,
                                     const T *const Lambda2,
                                     T *residual) const {
  // Form the transform from static camera to dynamic camera
  const Eigen::Matrix<T, 4, 4> T_sd =
      this->T_sd(tau_s, tau_d, w1, w2, Lambda1, Lambda2);

  // Calculate reprojection error by projecting 3D world point observed in
  // dynamic camera to static camera
  // -- Transform 3D world point from dynamic to static camera
  const Eigen::Matrix<T, 3, 1> P_d{T(this->P_d[0]),
                                   T(this->P_d[1]),
                                   T(this->P_d[2])};
  const Eigen::Matrix<T, 3, 1> P_s_cal = (T_sd * P_d.homogeneous()).head(3);
  // -- Project 3D world point to image plane
  const Eigen::Matrix<T, 3, 3> K_s =
      this->K(T(this->fx_s), T(this->fy_s), T(this->cx_s), T(this->cy_s));
  Eigen::Matrix<T, 3, 1> Q_s_cal = K_s * P_s_cal;
  // -- Normalize projected image point
  Q_s_cal(0) = Q_s_cal(0) / Q_s_cal(2);
  Q_s_cal(1) = Q_s_cal(1) / Q_s_cal(2);
  // // -- Calculate reprojection error
  residual[0] = T(this->Q_s[0]) - Q_s_cal(0);
  residual[1] = T(this->Q_s[1]) - Q_s_cal(1);

  // Calculate reprojection error by projecting 3D world point observed in
  // static camera to dynamic camera
  // -- Transform 3D world point from static to dynamic camera
  const Eigen::Matrix<T, 3, 1> P_s{T(this->P_s[0]),
                                   T(this->P_s[1]),
                                   T(this->P_s[2])};
  const Eigen::Matrix<T, 3, 1> P_d_cal =
      (T_sd.inverse() * P_s.homogeneous()).head(3);
  // -- Project 3D world point to image plane
  const Eigen::Matrix<T, 3, 3> K_d =
      this->K(T(this->fx_d), T(this->fy_d), T(this->cx_d), T(this->cy_d));
  Eigen::Matrix<T, 3, 1> Q_d_cal = K_d * P_d_cal;
  // -- Normalize projected image point
  Q_d_cal(0) = Q_d_cal(0) / Q_d_cal(2);
  Q_d_cal(1) = Q_d_cal(1) / Q_d_cal(2);
  // -- Calculate reprojection error
  residual[2] = T(this->Q_d[0]) - Q_d_cal(0);
  residual[3] = T(this->Q_d[1]) - Q_d_cal(1);

  return true;
}

} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_IMPL_RESIDUAL_HPP
