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
                                                        const T d,
                                                        const T a,
                                                        const T alpha) const {
  Eigen::Matrix<T, 4, 4> T_dh;

  T_dh(0, 0) = cos(theta);
  T_dh(0, 1) = -sin(theta) * cos(alpha);
  T_dh(0, 2) = sin(theta) * sin(alpha);
  T_dh(0, 3) = a * cos(theta);

  T_dh(1, 0) = sin(theta);
  T_dh(1, 1) = cos(theta) * cos(alpha);
  T_dh(1, 2) = -cos(theta) * sin(alpha);
  T_dh(1, 3) = a * sin(theta);

  T_dh(2, 0) = T(0.0);
  T_dh(2, 1) = sin(alpha);
  T_dh(2, 2) = cos(alpha);
  T_dh(2, 3) = d;

  T_dh(3, 0) = T(0.0);
  T_dh(3, 1) = T(0.0);
  T_dh(3, 2) = T(0.0);
  T_dh(3, 3) = T(1.0);

  return T_dh;
}

template <typename T>
Eigen::Matrix<T, 3, 3> GimbalCalibResidual::euler321ToRot(const T phi,
                                                          const T theta,
                                                          const T psi) const {
  // i.e. ZYX rotation sequence (world to body)
  const T R11 = cos(psi) * cos(theta);
  const T R12 = sin(psi) * cos(theta);
  const T R13 = -sin(theta);

  const T R21 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  const T R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  const T R23 = cos(theta) * sin(phi);

  const T R31 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  const T R32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  const T R33 = cos(theta) * cos(phi);

  Eigen::Matrix<T, 3, 3> R;
  // clang-format off
  R << R11, R21, R31,
       R12, R22, R32,
       R13, R23, R33;
  // clang-format on

  return R;
}

template <typename T>
Eigen::Matrix<T, 3, 3>
GimbalCalibResidual::K(const T fx, const T fy, const T cx, const T cy) const {
  Eigen::Matrix<T, 3, 3> K;

  K(0, 0) = fx;
  K(0, 1) = T(0.0);
  K(0, 2) = cx;

  K(1, 0) = T(0.0);
  K(1, 1) = fy;
  K(1, 2) = cy;

  K(2, 0) = T(0.0);
  K(2, 1) = T(0.0);
  K(2, 2) = T(1.0);

  return K;
}

template <typename T>
Eigen::Matrix<T, 4, 1>
GimbalCalibResidual::D(const T k1, const T k2, const T k3, const T k4) const {
  Eigen::Matrix<T, 4, 1> D;

  D(0) = k1;
  D(1) = k2;
  D(2) = k3;
  D(3) = k4;

  return D;
}

template <typename T>
Eigen::Matrix<T, 4, 4> GimbalCalibResidual::T_ds(const T *const tau_s,
                                                 const T *const tau_d,
                                                 const T *const w1,
                                                 const T *const w2,
                                                 const T *const Lambda1,
                                                 const T *const Lambda2) const {
  // Form T_bs
  Eigen::Matrix<T, 4, 4> T_bs;
  T_bs.block(0, 0, 3, 3) = this->euler321ToRot(tau_s[3], tau_s[4], tau_s[5]);
  T_bs.block(0, 3, 3, 1) = Eigen::Matrix<T, 3, 1>{tau_s[0], tau_s[1], tau_s[2]};
  T_bs(3, 3) = T(1.0);

  // Form T_eb
  // -- DH params for first link
  const T theta1 = Lambda1[0] - T(M_PI / 2.0);
  const T d1 = w1[0];
  const T a1 = w1[1];
  const T alpha1 = w1[2];
  // -- DH params for second link
  const T theta2 = Lambda2[0];
  const T d2 = w2[0];
  const T a2 = w2[1];
  const T alpha2 = w2[2];
  // -- Combine DH transforms to form T_eb
  // clang-format off
  const Eigen::Matrix<T, 4, 4> T_1b = this->dhTransform(theta1, d1, a1, alpha1).inverse();
  const Eigen::Matrix<T, 4, 4> T_e1 = this->dhTransform(theta2, d2, a2, alpha2).inverse();
  const Eigen::Matrix<T, 4, 4> T_eb = T_e1 * T_1b;
  // clang-format on

  // Form T_de
  Eigen::Matrix<T, 4, 4> T_de;
  T_de.block(0, 0, 3, 3) = this->euler321ToRot(tau_d[3], tau_d[4], tau_d[5]);
  T_de.block(0, 3, 3, 1) = Eigen::Matrix<T, 3, 1>{tau_d[0], tau_d[1], tau_d[2]};
  T_de(3, 3) = T(1.0);

  return T_de * T_eb * T_bs;
}

template <typename T>
Eigen::Matrix<T, 2, 1> GimbalCalibResidual::project_pinhole_equi(
    const Eigen::Matrix<T, 3, 3> &K,
    const Eigen::Matrix<T, 4, 1> &D,
    const Eigen::Matrix<T, 3, 1> &X) const {
  const T z = X(2);
  const T x = X(0) / z;
  const T y = X(1) / z;
  const T r = sqrt(pow(x, 2) + pow(y, 2));

  // Apply equi distortion
  const T theta = atan(r);
  const T th2 = pow(theta, 2);
  const T th4 = pow(theta, 4);
  const T th6 = pow(theta, 6);
  const T th8 = pow(theta, 8);
  const T k1 = D(0);
  const T k2 = D(1);
  const T k3 = D(2);
  const T k4 = D(3);
  const T thetad = theta * (T(1) + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const T x_dash = (thetad / r) * x;
  const T y_dash = (thetad / r) * y;

  // Project equi distorted point to image plane
  const T fx = K(0, 0);
  const T fy = K(1, 1);
  const T cx = K(0, 2);
  const T cy = K(1, 2);
  const Eigen::Matrix<T, 2, 1> pixel{fx * x_dash + cx, fy * y_dash + cy};

  return pixel;
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
  const Eigen::Matrix<T, 4, 4> T_ds =
      this->T_ds(tau_s, tau_d, w1, w2, Lambda1, Lambda2);

  // Calculate reprojection error by projecting 3D world point observed in
  // dynamic camera to static camera
  // -- Transform 3D world point from dynamic to static camera
  const Eigen::Matrix<T, 3, 1> P_d{T(this->P_d[0]),
                                   T(this->P_d[1]),
                                   T(this->P_d[2])};
  const Eigen::Matrix<T, 3, 1> P_s_cal =
      (T_ds.inverse() * P_d.homogeneous()).head(3);
  // -- Project 3D world point to image plane
  const Eigen::Matrix<T, 3, 3> K_s =
      this->K(T(this->fx_s), T(this->fy_s), T(this->cx_s), T(this->cy_s));
  const Eigen::Matrix<T, 4, 1> D_s =
      this->D(T(this->k1_s), T(this->k2_s), T(this->k3_s), T(this->k4_s));
  const Eigen::Matrix<T, 2, 1> Q_s_cal =
      this->project_pinhole_equi(K_s, D_s, P_s_cal);
  // // project_pinhole_equi(K_s,
  // Eigen::Matrix<T, 3, 1> Q_s_cal = K_s * P_s_cal;
  // // -- Normalize projected image point
  // Q_s_cal(0) = Q_s_cal(0) / Q_s_cal(2);
  // Q_s_cal(1) = Q_s_cal(1) / Q_s_cal(2);
  // // -- Calculate reprojection error
  residual[0] = T(this->Q_s[0]) - Q_s_cal(0);
  residual[1] = T(this->Q_s[1]) - Q_s_cal(1);

  // Calculate reprojection error by projecting 3D world point observed in
  // static camera to dynamic camera
  // -- Transform 3D world point from static to dynamic camera
  const Eigen::Matrix<T, 3, 1> P_s{T(this->P_s[0]),
                                   T(this->P_s[1]),
                                   T(this->P_s[2])};
  const Eigen::Matrix<T, 3, 1> P_d_cal = (T_ds * P_s.homogeneous()).head(3);
  // -- Project 3D world point to image plane
  const Eigen::Matrix<T, 3, 3> K_d =
      this->K(T(this->fx_d), T(this->fy_d), T(this->cx_d), T(this->cy_d));
  const Eigen::Matrix<T, 4, 1> D_d =
      this->D(T(this->k1_d), T(this->k2_d), T(this->k3_d), T(this->k4_d));
  const Eigen::Matrix<T, 2, 1> Q_d_cal =
      this->project_pinhole_equi(K_d, D_d, P_d_cal);
  // Eigen::Matrix<T, 3, 1> Q_d_cal = K_d * P_d_cal;
  // // -- Normalize projected image point
  // Q_d_cal(0) = Q_d_cal(0) / Q_d_cal(2);
  // Q_d_cal(1) = Q_d_cal(1) / Q_d_cal(2);
  // -- Calculate reprojection error
  residual[2] = T(this->Q_d[0]) - Q_d_cal(0);
  residual[3] = T(this->Q_d[1]) - Q_d_cal(1);

  return true;
}

} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_IMPL_RESIDUAL_HPP
