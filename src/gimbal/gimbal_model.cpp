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

Mat4 GimbalModel::T_bs() {
  const Vec3 t = this->tau_s.head(3);
  const Vec3 rpy = this->tau_s.tail(3);
  const Mat3 R = euler321ToRot(rpy);

  Mat4 T_sb;
  T_sb.block(0, 0, 3, 3) = R;
  T_sb.block(0, 3, 3, 1) = t;
  T_sb(3, 3) = 1.0;

  return T_sb;
}

Mat4 GimbalModel::T_eb() {
  const double theta1 = this->Lambda1;
  const double alpha1 = this->w1[0];
  const double a1 = this->w1[1];
  const double d1 = this->w1[2];

  const double theta2 = this->Lambda2;
  const double alpha2 = this->w2[0];
  const double a2 = this->w2[1];
  const double d2 = this->w2[2];

  const Mat4 T_1b = dh_transform(theta1, alpha1, a1, d1).inverse();
  const Mat4 T_e1 = dh_transform(theta2, alpha2, a2, d2).inverse();
  const Mat4 T_eb = T_e1 * T_1b;

  return T_eb;
}

Mat4 GimbalModel::T_de() {
  const Vec3 t = this->tau_d.head(3);
  const Vec3 rpy = this->tau_d.tail(3);
  const Mat3 R = euler321ToRot(rpy);

  Mat4 T_de;
  T_de.block(0, 0, 3, 3) = R;
  T_de.block(0, 3, 3, 1) = t;
  T_de(3, 3) = 1.0;

  return T_de;
}

Mat4 GimbalModel::T_ds() { return this->T_de() * this->T_eb() * this->T_bs(); }

} // namespace gvio
