#include "gvio/sim/twowheel.hpp"

namespace gvio {

void TwoWheelRobot::update(const double ax_B,
                           const double wz_B,
                           const double dt) {
  this->w_B = Vec3{0.0, 0.0, wz_B};
  this->a_B = Vec3{ax_B, 0.0, 0.0};
  this->v_B += this->a_B * dt;

  this->p_G(0) += v_B(0) * cos(this->rpy_G(2)) * dt;
  this->p_G(1) += v_B(1) * sin(this->rpy_G(2)) * dt;
  this->p_G(2) += v_B(2) * dt;
  this->rpy_G(2) += wz_B * dt;
}

} // namespace gvio
