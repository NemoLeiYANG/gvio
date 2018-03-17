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

Mat4 GimbalCalibNumericalResidual::T_ds(const VecX &tau_s,
                                        const double Lambda1,
                                        const Vec3 &w1,
                                        const VecX &tau_d,
                                        const double Lambda2,
                                        const Vec3 &w2) const {
  // Form T_sb
  Mat4 T_bs;
  T_bs.block(0, 0, 3, 3) = euler321ToRot({tau_s(3), tau_s(4), tau_s(5)});
  T_bs.block(0, 3, 3, 1) = Vec3{tau_s(0), tau_s(1), tau_s(2)};
  T_bs(3, 3) = 1.0;

  // Form T_eb
  // -- DH params for first link
  const double theta1 = Lambda1 - M_PI / 2.0;
  const double d1 = w1(0);
  const double a1 = w1(1);
  const double alpha1 = w1(2);
  // -- DH params for second link
  const double theta2 = Lambda2;
  const double d2 = w2(0);
  const double a2 = w2(1);
  const double alpha2 = w2(2);
  // -- Combine DH transforms to form T_eb
  const Mat4 T_1b = dh_transform(theta1, d1, a1, alpha1).inverse();
  const Mat4 T_e1 = dh_transform(theta2, d2, a2, alpha2).inverse();
  const Mat4 T_eb = T_e1 * T_1b;

  // Form T_ed
  Mat4 T_de;
  T_de.block(0, 0, 3, 3) = euler321ToRot({tau_d(3), tau_d(4), tau_d(5)});
  T_de.block(0, 3, 3, 1) = Vec3{tau_d(0), tau_d(1), tau_d(2)};
  T_de(3, 3) = 1.0;

  return T_de * T_eb * T_bs;
}

bool GimbalCalibNumericalResidual::operator()(const double *const p0,
                                              const double *const p1,
                                              const double *const p2,
                                              const double *const p3,
                                              const double *const p4,
                                              const double *const p5,
                                              double *residual) const {
  // Map stacked optimization parameters back to its respective parameter
  // -- tau_s
  VecX tau_s = zeros(6, 1);
  tau_s << p0[0], p0[1], p0[2], p0[3], p0[4], p0[5];
  // -- tau_d
  VecX tau_d = zeros(6, 1);
  tau_d << p1[0], p1[1], p1[2], p1[3], p1[4], p1[5];
  // -- First DH link
  const double Lambda1 = p4[0];
  const Vec3 w1{p2[0], p2[1], p2[2]};
  // -- Second DH link
  const double Lambda2 = p5[0];
  const Vec3 w2{p3[0], p3[1], p3[2]};

  // Form the transform from static camera to dynamic camera
  const Mat4 T_ds = this->T_ds(tau_s, Lambda1, w1, tau_d, Lambda2, w2);

  // Calculate reprojection error by projecting 3D world point observed in
  // dynamic camera to static camera
  // -- Transform 3D world point from dynamic to static camera
  const Vec3 P_s_cal = (T_ds.inverse() * this->P_d.homogeneous()).head(3);
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
  const Vec3 P_d_cal = (T_ds * this->P_s.homogeneous()).head(3);
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
