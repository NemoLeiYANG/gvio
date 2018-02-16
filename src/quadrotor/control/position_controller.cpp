#include "gvio/quadrotor/control/position_controller.hpp"

namespace gvio {

// int PositionController::configure(const std::string &config_file) {
//   ConfigParser parser;
//
//   // load config
//   parser.addParam("roll_controller.k_p", &this->y_controller.k_p);
//   parser.addParam("roll_controller.k_i", &this->y_controller.k_i);
//   parser.addParam("roll_controller.k_d", &this->y_controller.k_d);
//   parser.addParam("roll_controller.min", &this->roll_limit[0]);
//   parser.addParam("roll_controller.max", &this->roll_limit[1]);
//
//   parser.addParam("pitch_controller.k_p", &this->x_controller.k_p);
//   parser.addParam("pitch_controller.k_i", &this->x_controller.k_i);
//   parser.addParam("pitch_controller.k_d", &this->x_controller.k_d);
//   parser.addParam("pitch_controller.min", &this->pitch_limit[0]);
//   parser.addParam("pitch_controller.max", &this->pitch_limit[1]);
//
//   parser.addParam("throttle_controller.k_p", &this->z_controller.k_p);
//   parser.addParam("throttle_controller.k_i", &this->z_controller.k_i);
//   parser.addParam("throttle_controller.k_d", &this->z_controller.k_d);
//   parser.addParam("throttle_controller.hover_throttle",
//   &this->hover_throttle);
//
//   if (parser.load(config_file) != 0) {
//     return -1;
//   }
//
//   // convert roll and pitch limits from degrees to radians
//   this->roll_limit[0] = deg2rad(this->roll_limit[0]);
//   this->roll_limit[1] = deg2rad(this->roll_limit[1]);
//   this->pitch_limit[0] = deg2rad(this->pitch_limit[0]);
//   this->pitch_limit[1] = deg2rad(this->pitch_limit[1]);
//
//   this->configured = true;
//   return 0;
// }

Vec4 PositionController::update(const Vec3 &setpoints,
                                const Vec4 &actual,
                                double yaw,
                                double dt) {
  // Check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // Calculate RPY errors relative to quadrotor by incorporating yaw
  Vec3 errors{setpoints(0) - actual(0),
              setpoints(1) - actual(1),
              setpoints(2) - actual(2)};
  const Vec3 euler{0.0, 0.0, actual(3)};
  const Mat3 R = euler123ToRot(euler);
  errors = R * errors;

  // Roll, pitch, yaw and thrust
  double r = -this->y_controller.update(errors(1), dt);
  double p = this->x_controller.update(errors(0), dt);
  double y = yaw;
  double t = 0.5 + this->z_controller.update(errors(2), dt);
  outputs << r, p, y, t;

  // Limit roll, pitch
  for (int i = 0; i < 2; i++) {
    if (outputs(i) > deg2rad(30.0)) {
      outputs(i) = deg2rad(30.0);
    } else if (outputs(i) < deg2rad(-30.0)) {
      outputs(i) = deg2rad(-30.0);
    }
  }

  // Limit yaw
  while (outputs(2) > deg2rad(360.0)) {
    outputs(2) -= deg2rad(360.0);
  }
  while (outputs(2) < deg2rad(0.0)) {
    outputs(2) += deg2rad(360.0);
  }

  // Limit thrust
  if (outputs(3) > 1.0) {
    outputs(3) = 1.0;
  } else if (outputs(3) < 0.0) {
    outputs(3) = 0.0;
  }

  // Yaw first if threshold reached
  if (fabs(yaw - actual(3)) > deg2rad(2)) {
    outputs(0) = 0.0;
    outputs(1) = 0.0;
  }

  // Keep track of outputs
  this->outputs = outputs;
  this->dt = 0.0;

  return outputs;
}

void PositionController::reset() {
  this->x_controller.reset();
  this->y_controller.reset();
  this->z_controller.reset();
}

void PositionController::printOutputs() {
  double r = rad2deg(this->outputs(0));
  double p = rad2deg(this->outputs(1));
  double t = this->outputs(3);

  std::cout << "roll: " << std::setprecision(2) << r << "\t";
  std::cout << "pitch: " << std::setprecision(2) << p << "\t";
  std::cout << "throttle: " << std::setprecision(2) << t << std::endl;
}

} // namespace gvio
