#include "gvio/quadrotor/control/position_controller.hpp"

namespace gvio {

Vec4 PositionController::update(const Vec3 &setpoints,
                                const Vec4 &actual,
                                double yaw,
                                double dt) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // calculate RPY errors relative to quadrotor by incorporating yaw
  Vec3 errors;
  errors(0) = setpoints(0) - actual(0);
  errors(1) = setpoints(1) - actual(1);
  errors(2) = setpoints(2) - actual(2);

  Vec3 euler{0.0, 0.0, actual(3)};
  Mat3 R = euler123ToRot(euler);
  errors = R * errors;

  // roll, pitch, yaw and thrust
  double r = -this->y_controller.update(errors(1), 0.0, dt);
  double p = this->x_controller.update(errors(0), 0.0, dt);
  double y = yaw;
  double t = 0.5 + this->z_controller.update(errors(2), 0.0, dt);
  outputs << r, p, y, t;

  // limit roll, pitch
  for (int i = 0; i < 2; i++) {
    if (outputs(i) > deg2rad(30.0)) {
      outputs(i) = deg2rad(30.0);
    } else if (outputs(i) < deg2rad(-30.0)) {
      outputs(i) = deg2rad(-30.0);
    }
  }

  // limit yaw
  while (outputs(2) > deg2rad(360.0)) {
    outputs(2) -= deg2rad(360.0);
  }
  while (outputs(2) < deg2rad(0.0)) {
    outputs(2) += deg2rad(360.0);
  }

  // limit thrust
  if (outputs(3) > 1.0) {
    outputs(3) = 1.0;
  } else if (outputs(3) < 0.0) {
    outputs(3) = 0.0;
  }

  // yaw first if threshold reached
  if (fabs(yaw - actual(3)) > deg2rad(2)) {
    outputs(0) = 0.0;
    outputs(1) = 0.0;
  }

  // keep track of outputs
  this->outputs = outputs;
  this->dt = 0.0;

  return outputs;
}

} // namespace gvio
