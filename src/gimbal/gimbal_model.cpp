#include "gvio/gimbal/gimbal_model.hpp"

namespace gvio {

Mat4 dh_transform(const double theta,
                  const double d,
                  const double a,
                  const double alpha) {
  // clang-format off
  Mat4 T;
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
       sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
       0.0, sin(alpha), cos(alpha), d,
       0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return T;
}

GimbalModel::GimbalModel() {}

GimbalModel::~GimbalModel() {}

void GimbalModel::setAttitude(const double roll, const double pitch) {
  this->attitude << roll, pitch;
}

Mat4 GimbalModel::T_bs() {
  Mat4 T_sb = zeros(4, 4);
  T_sb.block(0, 0, 3, 3) = euler321ToRot(this->tau_s.tail(3));
  T_sb.block(0, 3, 3, 1) = this->tau_s.head(3);
  T_sb(3, 3) = 1.0;

  return T_sb;
}

Mat4 GimbalModel::T_eb() {
  const double theta1 = this->Lambda1 + this->theta1_offset;
  const double d1 = this->w1[0];
  const double a1 = this->w1[1];
  const double alpha1 = this->w1[2];

  const double theta2 = this->Lambda2 + this->theta2_offset;
  const double d2 = this->w2[0];
  const double a2 = this->w2[1];
  const double alpha2 = this->w2[2];

  const Mat4 T_1b = dh_transform(theta1, d1, a1, alpha1).inverse();
  const Mat4 T_e1 = dh_transform(theta2, d2, a2, alpha2).inverse();
  const Mat4 T_eb = T_e1 * T_1b;

  return T_eb;
}

Mat4 GimbalModel::T_de() {
  Mat4 T_de = zeros(4, 4);
  T_de.block(0, 0, 3, 3) = euler321ToRot(this->tau_d.tail(3));
  T_de.block(0, 3, 3, 1) = this->tau_d.head(3);
  T_de(3, 3) = 1.0;

  return T_de;
}

Mat4 GimbalModel::T_ds() { return this->T_de() * this->T_eb() * this->T_bs(); }

std::ostream &operator<<(std::ostream &os, const GimbalModel &gimbal) {
  os << "tau_s: " << gimbal.tau_s.transpose() << std::endl;
  os << "tau_d: " << gimbal.tau_d.transpose() << std::endl;
  os << "w1: " << gimbal.w1.transpose() << std::endl;
  os << "w2: " << gimbal.w2.transpose() << std::endl;
  os << "Lambda1: " << gimbal.Lambda1 << std::endl;
  os << "Lambda2: " << gimbal.Lambda2 << std::endl;
  os << "theta1_offset: " << gimbal.theta1_offset << std::endl;
  os << "theta2_offset: " << gimbal.theta2_offset << std::endl;
  return os;
}

} // namespace gvio
