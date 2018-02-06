#include "gvio/sim/quadrotor.hpp"

namespace gvio {

// ATTITUDE CONTROLLER
Vec4 AttitudeController::update(const Vec4 &setpoints,
                                const Vec4 &actual,
                                double dt) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.001) {
    return this->outputs;
  }

  // calculate yaw error
  double actual_yaw = rad2deg(actual(2));
  double setpoint_yaw = rad2deg(setpoints(2));
  double error_yaw = setpoint_yaw - actual_yaw;
  if (error_yaw > 180.0) {
    error_yaw -= 360.0;
  } else if (error_yaw < -180.0) {
    error_yaw += 360.0;
  }
  error_yaw = deg2rad(error_yaw);

  // roll pitch yaw
  double r = this->roll_controller.update(setpoints(0), actual(0), this->dt);
  double p = this->pitch_controller.update(setpoints(1), actual(1), this->dt);
  double y = this->yaw_controller.update(error_yaw, 0.0, this->dt);

  // thrust
  double max_thrust = 5.0;
  double t = max_thrust * setpoints(3);  // convert relative to true thrust
  t = (t > max_thrust) ? max_thrust : t; // limit thrust
  t = (t < 0) ? 0.0 : t;                 // limit thrust

  // map roll, pitch, yaw and thrust to motor outputs
  Vec4 outputs;
  outputs(0) = -p - y + t;
  outputs(1) = -r + y + t;
  outputs(2) = p - y + t;
  outputs(3) = r + y + t;

  // limit outputs
  for (int i = 0; i < 4; i++) {
    if (outputs(i) > max_thrust) {
      outputs(i) = max_thrust;
    } else if (outputs(i) < 0.0) {
      outputs(i) = 0.0;
    }
  }

  // keep track of outputs
  this->outputs = outputs;
  this->dt = 0.0;

  return outputs;
}

// POSITION CONTROLLER
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

// QUADROTOR MODEL
int QuadrotorModel::update(const VecX &motor_inputs, const double dt) {
  const double ph = this->rpy_G(0);
  const double th = this->rpy_G(1);
  const double ps = this->rpy_G(2);

  const double p = this->w_G(0);
  const double q = this->w_G(1);
  const double r = this->w_G(2);

  const double x = this->p_G(0);
  const double y = this->p_G(1);
  const double z = this->p_G(2);

  const double vx = this->v_G(0);
  const double vy = this->v_G(1);
  const double vz = this->v_G(2);

  const double Ix = this->Ix;
  const double Iy = this->Iy;
  const double Iz = this->Iz;

  const double kr = this->kr;
  const double kt = this->kt;

  const double m = this->m;
  const double g = this->g;

  // convert motor inputs to angular p, q, r and total thrust
  // clang-format off
  Mat4 A;
  A << 1.0, 1.0, 1.0, 1.0,
        0.0, -this->l, 0.0, this->l,
        -this->l, 0.0, this->l, 0.0,
        -this->d, this->d, -this->d, this->d;
  // clang-format on

  Vec4 tau = A * motor_inputs;
  const double tauf = tau(0);
  const double taup = tau(1);
  const double tauq = tau(2);
  const double taur = tau(3);

  // update
  // clang-format off
  this->rpy_G(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
  this->rpy_G(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
  this->rpy_G(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;

  this->w_G(0) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
  this->w_G(1) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
  this->w_G(2) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;

  this->p_G(0) = x + vx * dt;
  this->p_G(1) = y + vy * dt;
  this->p_G(2) = z + vz * dt;

	this->a_G(0) = ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
	this->a_G(1) = ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
	this->a_G(2) = (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;

  this->v_G(0) = vx + this->a_G(0);
  this->v_G(1) = vy + this->a_G(1);
  this->v_G(2) = vz + this->a_G(2);
  // clang-format on

  // constrain yaw to be [-180, 180]
  this->rpy_G(2) = wrapToPi(this->rpy_G(2));

  return 0;
}

int QuadrotorModel::update(const double dt) {
  Vec4 motor_inputs;
  if (this->ctrl_mode == "POS_CTRL_MODE") {
    motor_inputs = this->positionControllerControl(dt);
  } else if (this->ctrl_mode == "ATT_CTRL_MODE") {
    motor_inputs = this->attitudeControllerControl(dt);
  }

  return this->update(motor_inputs, dt);
}

Vec4 QuadrotorModel::attitudeControllerControl(const double dt) {
  const Vec4 actual_attitude{this->rpy_G(0), // roll
                             this->rpy_G(1), // pitch
                             this->rpy_G(2), // yaw
                             this->p_G(2)};  // z

  const Vec4 motor_inputs =
      this->attitude_controller.update(this->attitude_setpoints,
                                       actual_attitude,
                                       dt);

  return motor_inputs;
}

Vec4 QuadrotorModel::positionControllerControl(const double dt) {
  // Position controller
  Vec4 actual_position;
  actual_position(0) = this->p_G(0);   // x
  actual_position(1) = this->p_G(1);   // y
  actual_position(2) = this->p_G(2);   // z
  actual_position(3) = this->rpy_G(2); // yaw

  this->attitude_setpoints =
      this->position_controller.update(this->position_setpoints,
                                       actual_position,
                                       0.0,
                                       dt);

  // Attitude controller
  Vec4 actual_attitude;
  actual_attitude(0) = this->rpy_G(0); // roll
  actual_attitude(1) = this->rpy_G(1); // pitch
  actual_attitude(2) = this->rpy_G(2); // yaw
  actual_attitude(3) = this->p_G(2);   // z

  const Vec4 motor_inputs =
      this->attitude_controller.update(this->attitude_setpoints,
                                       actual_attitude,
                                       dt);

  return motor_inputs;
}

void QuadrotorModel::setAttitude(const double roll,
                                 const double pitch,
                                 const double yaw,
                                 const double z) {
  this->attitude_setpoints(0) = roll;
  this->attitude_setpoints(1) = pitch;
  this->attitude_setpoints(2) = yaw;
  this->attitude_setpoints(3) = z;
}

void QuadrotorModel::setPosition(const double x,
                                 const double y,
                                 const double z) {
  this->position_setpoints(0) = x;
  this->position_setpoints(1) = y;
  this->position_setpoints(2) = z;
}

VecX QuadrotorModel::getPose() {
  VecX pose(6);

  // x, y, z
  pose(0) = this->p_G(0);
  pose(1) = this->p_G(1);
  pose(2) = this->p_G(2);

  // phi, theta, psi
  pose(3) = this->rpy_G(0);
  pose(4) = this->rpy_G(1);
  pose(5) = this->rpy_G(2);

  return pose;
}

Vec3 QuadrotorModel::getVelocity() { return this->v_G; }

Vec3 QuadrotorModel::getAngularVelocity() { return this->w_G; }

Vec3 QuadrotorModel::getBodyAngularVelocity() {
  const Mat3 R_BG = euler123ToRot(this->rpy_G);
  const Vec3 w_B = R_BG * this->w_G;
  return w_B;
}

Vec3 QuadrotorModel::getBodyAcceleration() {
  const Mat3 R_BG = euler123ToRot(this->rpy_G);
  const Vec3 a_B = R_BG * this->a_G;
  return a_B;
}

void QuadrotorModel::printState() {
  printf("x: %f\t", this->p_G(0));
  printf("y: %f\t", this->p_G(1));
  printf("z: %f\t\t", this->p_G(2));

  printf("phi: %f\t", this->rpy_G(0));
  printf("theta: %f\t", this->rpy_G(1));
  printf("psi: %f\n", this->rpy_G(2));
}

} // namespace gvio
