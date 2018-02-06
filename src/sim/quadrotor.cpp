#include "gvio/sim/quadrotor.hpp"

namespace gvio {

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

Vec4 AttitudeController::update(const Vec4 &psetpoints,
                                const Vec4 &vsetpoints,
                                const Vec4 &actual,
                                double dt) {
  Vec4 setpoints;
  setpoints = psetpoints + vsetpoints;
  return this->update(setpoints, actual, dt);
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
  const double ph = this->attitude(0);
  const double th = this->attitude(1);
  const double ps = this->attitude(2);

  const double p = this->angular_velocity(0);
  const double q = this->angular_velocity(1);
  const double r = this->angular_velocity(2);

  const double x = this->position(0);
  const double y = this->position(1);
  const double z = this->position(2);

  const double vx = this->linear_velocity(0);
  const double vy = this->linear_velocity(1);
  const double vz = this->linear_velocity(2);

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
  this->attitude(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
  this->attitude(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
  this->attitude(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;

  this->angular_velocity(0) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
  this->angular_velocity(1) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
  this->angular_velocity(2) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;

  this->position(0) = x + vx * dt;
  this->position(1) = y + vy * dt;
  this->position(2) = z + vz * dt;

	const double ax = ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
	const double ay = ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
	const double az = (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;

  this->linear_velocity(0) = vx + ax;
  this->linear_velocity(1) = vy + ay;
  this->linear_velocity(2) = vz + az;
  // clang-format on

  // constrain yaw to be [-180, 180]
  this->attitude(2) = wrapToPi(this->attitude(2));

  return 0;
}

Vec4 QuadrotorModel::attitudeControllerControl(const double dt) {
  const Vec4 actual_attitude{this->attitude(0),  // roll
                             this->attitude(1),  // pitch
                             this->attitude(2),  // yaw
                             this->position(2)}; // z

  const Vec4 motor_inputs =
      this->attitude_controller.update(this->attitude_setpoints,
                                       actual_attitude,
                                       dt);

  return motor_inputs;
}

Vec4 QuadrotorModel::positionControllerControl(const double dt) {
  // position controller
  Vec4 actual_position;
  actual_position(0) = this->position(0); // x
  actual_position(1) = this->position(1); // y
  actual_position(2) = this->position(2); // z
  actual_position(3) = this->attitude(2); // yaw

  this->attitude_setpoints =
      this->position_controller.update(this->position_setpoints,
                                       actual_position,
                                       0.0,
                                       dt);

  // attitude controller
  Vec4 actual_attitude;
  actual_attitude(0) = this->attitude(0); // roll
  actual_attitude(1) = this->attitude(1); // pitch
  actual_attitude(2) = this->attitude(2); // yaw
  actual_attitude(3) = this->position(2); // z

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
  pose(0) = this->position(0);
  pose(1) = this->position(1);
  pose(2) = this->position(2);

  // phi, theta, psi
  pose(3) = this->attitude(0);
  pose(4) = this->attitude(1);
  pose(5) = this->attitude(2);

  return pose;
}

Vec3 QuadrotorModel::getVelocity() {
  // vx, vy, vz
  const Vec3 vel{this->linear_velocity(0),
                 this->linear_velocity(1),
                 this->linear_velocity(2)};

  return vel;
}

Vec3 QuadrotorModel::getAngularVelocity() {
  // phi_dot, theta_dot, psi_dot
  const Vec3 avel{this->angular_velocity(0),
                  this->angular_velocity(1),
                  this->angular_velocity(2)};

  return avel;
}

void QuadrotorModel::printState() {
  printf("x: %f\t", this->position(0));
  printf("y: %f\t", this->position(1));
  printf("z: %f\t\t", this->position(2));

  printf("phi: %f\t", this->attitude(0));
  printf("theta: %f\t", this->attitude(1));
  printf("psi: %f\n", this->attitude(2));
}

} // namespace gvio
