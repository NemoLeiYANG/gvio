#include "gvio/gimbal/gimbal_model.hpp"

namespace gvio {

Mat4 dh_transform(const double theta,
                  const double alpha,
                  const double a,
                  const double d) {
  // clang-format off
  Mat4 T;
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
       sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
       0.0, sin(alpha), cos(alpha), d,
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T;
}

void GimbalModel::setAttitude(const double roll, const double pitch) {
  this->attitude << roll, pitch;
}

Mat4 GimbalModel::T_sb() {
  const Vec3 t_G_sb = this->tau_s.head(3);
  const Vec3 rpy_bs = this->tau_s.tail(3);
  const Mat3 R_sb = euler321ToRot(rpy_bs);

  // Create base frame
  // clang-format off
  Mat4 T_sb;
  T_sb << R_sb(0, 0), R_sb(0, 1), R_sb(0, 2), t_G_sb(0),
          R_sb(1, 0), R_sb(1, 1), R_sb(1, 2), t_G_sb(1),
          R_sb(2, 0), R_sb(2, 1), R_sb(2, 2), t_G_sb(2),
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T_sb;
}

Mat4 GimbalModel::T_be() {
  const double theta1 = this->Lambda1;
  const double alpha1 = this->w1[0];
  const double a1 = this->w1[1];
  const double d1 = this->w1[2];

  const double theta2 = this->Lambda2;
  const double alpha2 = this->w2[0];
  const double a2 = this->w2[1];
  const double d2 = this->w2[2];

  const Mat4 T_b1 = dh_transform(theta1, alpha1, a1, d1);
  const Mat4 T_1e = dh_transform(theta2, alpha2, a2, d2);
  const Mat4 T_be = T_b1 * T_1e;

  return T_be;
}

Mat4 GimbalModel::T_ed() {
  const Vec3 t_d_ed = this->tau_d.head(3);
  const Vec3 rpy_de = this->tau_d.tail(3);
  const Mat3 R_ed = euler321ToRot(rpy_de);

  // Create base frame
  // clang-format off
  Mat4 T_ed;
  T_ed << R_ed(0, 0), R_ed(0, 1), R_ed(0, 2), t_d_ed(0),
          R_ed(1, 0), R_ed(1, 1), R_ed(1, 2), t_d_ed(1),
          R_ed(2, 0), R_ed(2, 1), R_ed(2, 2), t_d_ed(2),
          0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T_ed;
}

Mat4 GimbalModel::T_sd() {
  // Transform from static camera to base frame
  const Mat4 T_sb = this->T_sb();

  // Transform from base frame to end-effector
  const Mat4 T_be = this->T_be();

  // Transform from end-effector to dynamic camera
  const Mat4 T_ed = this->T_ed();

  // Combine transforms
  const Mat4 T_se = T_sb * T_be; // Transform static camera to end effector
  const Mat4 T_sd = T_se * T_ed; // Transform static camera to dynamic camera

  return T_sd;
}

} // namespace gvio
