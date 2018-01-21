#include "gvio/sim/twowheel.hpp"

namespace gvio {

void circle_trajectory(const double r,
                       const double v,
                       double *w,
                       double *time) {
  const double dist = 2 * M_PI * r;
  *time = dist / v;
  *w = (2 * M_PI) / *time;
}

TwoWheelRobot::TwoWheelRobot() {}

TwoWheelRobot::TwoWheelRobot(const Vec3 &p_G,
                             const Vec3 &v_G,
                             const Vec3 &rpy_G) {
  this->p_G = p_G;
  this->v_G = v_G;
  this->rpy_G = rpy_G;
}

TwoWheelRobot::~TwoWheelRobot() {}

void TwoWheelRobot::update(const double dt) {
  const Vec3 p_G_prev = this->p_G;
  const Vec3 v_G_prev = this->v_G;
  const Vec3 rpy_G_prev = this->rpy_G;

  this->p_G += euler321ToRot(this->rpy_G) * this->v_B * dt;
  this->v_G = (this->p_G - p_G_prev) / dt;
  this->a_G = (this->v_G - v_G_prev) / dt;

  this->rpy_G += euler321ToRot(this->rpy_G) * this->w_B * dt;
  this->w_G = this->rpy_G - rpy_G_prev;
  this->a_B = euler123ToRot(this->rpy_G) * this->a_G;

  // Wrap angles to +/- pi
  for (int i = 0; i < 3; i++) {
    this->rpy_G(i) =
        (this->rpy_G(i) > M_PI) ? this->rpy_G(i) - 2 * M_PI : this->rpy_G(i);
    this->rpy_G(i) =
        (this->rpy_G(i) < -M_PI) ? this->rpy_G(i) + 2 * M_PI : this->rpy_G(i);
  }
}

} // namespace gvio
