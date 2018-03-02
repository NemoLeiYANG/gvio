#include "gvio/gimbal/calibration/residual.hpp"

namespace gvio {

GimbalCalibResidual::GimbalCalibResidual() {}

GimbalCalibResidual::GimbalCalibResidual(const Vec3 &P_s,
                                         const Vec3 &P_d,
                                         const Vec2 &Q_s,
                                         const Vec2 &Q_d,
                                         const Mat3 &K_s,
                                         const Mat3 &K_d) {
  // Observed 3d point in static camera
  this->P_s[0] = P_s(0);
  this->P_s[1] = P_s(1);
  this->P_s[2] = P_s(2);

  // Observed 3d point in dynamic camera
  this->P_d[0] = P_d(0);
  this->P_d[1] = P_d(1);
  this->P_d[2] = P_d(2);

  // Observed pixel in static camera
  this->Q_s[0] = Q_s(0);
  this->Q_s[1] = Q_s(1);

  // Observed pixel in dynamic camera
  this->Q_d[0] = Q_d(0);
  this->Q_d[1] = Q_d(1);

  // Static camera intrinsics
  this->fx_s = K_s(0, 0);
  this->fy_s = K_s(1, 1);
  this->cx_s = K_s(0, 2);
  this->cy_s = K_s(1, 2);

  // Dynamic camera intrinsics
  this->fx_d = K_d(0, 0);
  this->fy_d = K_d(1, 1);
  this->cx_d = K_d(0, 2);
  this->cy_d = K_d(1, 2);
}

std::ostream &operator<<(std::ostream &os,
                         const GimbalCalibResidual &residual) {
  os << "P_s: " << array2str(residual.P_s, 3) << std::endl;
  os << "P_d: " << array2str(residual.P_d, 3) << std::endl;
  os << "Q_s: " << array2str(residual.Q_s, 2) << std::endl;
  os << "Q_d: " << array2str(residual.Q_d, 2) << std::endl;
  return os;
}

std::ostream &operator<<(std::ostream &os,
                         const GimbalCalibResidual *residual) {
  os << "P_s: " << array2str(residual->P_s, 3) << std::endl;
  os << "P_d: " << array2str(residual->P_d, 3) << std::endl;
  os << "Q_s: " << array2str(residual->Q_s, 2) << std::endl;
  os << "Q_d: " << array2str(residual->Q_d, 2) << std::endl;
  return os;
}

Mat4 GimbalCalibNumericalResidual::T_sd(const Vec3 tau_s,
                                        const double Lambda1,
                                        const Vec3 w1,
                                        const Vec3 tau_d,
                                        const double Lambda2,
                                        const Vec3 w2) const {
  // Form T_sb
  const Vec3 t_G_sb{tau_s(0), tau_s(1), tau_s(2)};
  const Vec3 rpy_bs{tau_s(3), tau_s(4), tau_s(5)};
  const Mat3 R_sb = euler321ToRot(rpy_bs);
  // clang-format off
  Mat4 T_sb;
  T_sb << R_sb(0, 0), R_sb(0, 1), R_sb(0, 2), t_G_sb(0),
          R_sb(1, 0), R_sb(1, 1), R_sb(1, 2), t_G_sb(1),
          R_sb(2, 0), R_sb(2, 1), R_sb(2, 2), t_G_sb(2),
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Form T_eb
  // -- DH params for first link
  const double theta1 = Lambda1;
  const double alpha1 = w1(0);
  const double a1 = w1(1);
  const double d1 = w1(2);
  // -- DH params for second link
  const double theta2 = Lambda2;
  const double alpha2 = w2(0);
  const double a2 = w2(1);
  const double d2 = w2(2);
  // -- Combine DH transforms to form T_eb
  // clang-format off
  const Mat4 T_b1 = dh_transform(theta1, alpha1, a1, d1);
  const Mat4 T_1e = dh_transform(theta2, alpha2, a2, d2);
  const Mat4 T_be = T_b1 * T_1e;
  // clang-format on

  // Form T_ed
  const Vec3 t_d_ed{tau_d(0), tau_d(1), tau_d(2)};
  const Vec3 rpy_de{tau_d(3), tau_d(4), tau_d(5)};
  const Mat3 R_ed = euler321ToRot(rpy_de);
  // clang-format off
  Mat4 T_ed;
  T_ed << R_ed(0, 0), R_ed(0, 1), R_ed(0, 2), t_d_ed(0),
          R_ed(1, 0), R_ed(1, 1), R_ed(1, 2), t_d_ed(1),
          R_ed(2, 0), R_ed(2, 1), R_ed(2, 2), t_d_ed(2),
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T_sb * T_be * T_ed;
}

bool GimbalCalibNumericalResidual::operator()(const double *const p0,
                                              const double *const p1,
                                              const double *const p2,
                                              const double *const p3,
                                              const double *const p4,
                                              const double *const p5,
                                              double *residual) const {
  // Map stacked optimization parameters back to its respective parameter
  const Vec3 tau_s{p0[0], p0[1], p0[2]};
  const double Lambda1 = p1[0];
  const Vec3 w1{p2[0], p2[1], p2[2]};
  const Vec3 tau_d{p3[0], p3[1], p3[2]};
  const double Lambda2 = p4[0];
  const Vec3 w2{p5[0], p5[1], p5[2]};

  // Form the transform from static camera to dynamic camera
  const Mat4 T_sd = this->T_sd(tau_s, Lambda1, w1, tau_d, Lambda2, w2);

  // Calculate reprojection error by projecting 3D world point observed in
  // dynamic camera to static camera
  // -- Transform 3D world point from dynamic to static camera
  const Vec3 P_s_cal = (T_sd * this->P_d.homogeneous()).head(3);
  // -- Project 3D world point to image plane
  Vec3 Q_s_cal = this->K_s * P_s_cal;
  // -- Normalize projected image point
  Q_s_cal(0) = Q_s_cal(0) / Q_s_cal(2);
  Q_s_cal(1) = Q_s_cal(1) / Q_s_cal(2);
  // // -- Calculate reprojection error
  residual[0] = this->Q_s(0) - Q_s_cal(0);
  residual[1] = this->Q_s(1) - Q_s_cal(1);

  // Calculate reprojection error by projecting 3D world point observed in
  // static camera to dynamic camera
  // -- Transform 3D world point from dynamic to static camera
  const Vec3 P_d_cal = (T_sd.inverse() * this->P_s.homogeneous()).head(3);
  // -- Project 3D world point to image plane
  Vec3 Q_d_cal = this->K_d * P_d_cal;
  // -- Normalize projected image point
  Q_d_cal(0) = Q_d_cal(0) / Q_d_cal(2);
  Q_d_cal(1) = Q_d_cal(1) / Q_d_cal(2);
  // -- Calculate reprojection error
  residual[2] = this->Q_d(0) - Q_d_cal(0);
  residual[3] = this->Q_d(1) - Q_d_cal(1);

  return true;
}

} // namespace gvio
