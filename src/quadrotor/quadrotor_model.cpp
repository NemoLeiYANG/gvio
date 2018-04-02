#include "gvio/quadrotor/quadrotor_model.hpp"

namespace gvio {

int QuadrotorModel::loadMission(const std::string &mission_file) {
  this->ctrl_mode = "WP_CTRL_MODE";
  if (this->mission.configure(mission_file) != 0) {
    return -1;
  }

  return 0;
}

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

  // Update
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

	this->a_G(0) = ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf);
	this->a_G(1) = ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf);
	this->a_G(2) = (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g);

  this->v_G(0) = vx + this->a_G(0) * dt;
  this->v_G(1) = vy + this->a_G(1) * dt;
  this->v_G(2) = vz + this->a_G(2) * dt;
  // clang-format on

  // Constrain yaw to be [-180, 180]
  this->rpy_G(2) = wrapToPi(this->rpy_G(2));

  // Calculate body acceleration and angular velocity
  const Mat3 R_BG = euler321ToRot(this->rpy_G);
  const Vec3 g_B = euler321ToRot(this->rpy_G) * Vec3{0.0, 0.0, this->g};
  this->w_B = R_BG * this->w_G;
  this->a_B = (R_BG * this->a_G) + g_B;

  return 0;
}

int QuadrotorModel::update(const double dt) {
  Vec4 motor_inputs;
  if (this->ctrl_mode == "POS_CTRL_MODE") {
    motor_inputs = this->positionControllerControl(dt);

  } else if (this->ctrl_mode == "ATT_CTRL_MODE") {
    motor_inputs = this->attitudeControllerControl(dt);

  } else if (this->ctrl_mode == "WP_CTRL_MODE") {
    if (this->mission.configured == false) {
      LOG_ERROR("Mission is not configured!");
      return -1;
    }

    motor_inputs = this->waypointControllerControl(dt);
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
  const Vec4 actual_position{this->p_G(0),    // x
                             this->p_G(1),    // y
                             this->p_G(2),    // z
                             this->rpy_G(2)}; // yaw
  this->attitude_setpoints =
      this->position_controller.update(this->position_setpoints,
                                       actual_position,
                                       0.0,
                                       dt);

  // Attitude controller
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

Vec4 QuadrotorModel::waypointControllerControl(const double dt) {
  // Waypoint controller
  int retval = this->waypoint_controller.update(this->mission,
                                                this->p_G,
                                                this->v_G,
                                                this->rpy_G,
                                                dt);
  if (retval != 0) {
    this->attitude_setpoints = Vec4{0.0, 0.0, 0.0, 0.5};
  } else {
    this->attitude_setpoints = this->waypoint_controller.outputs;
  }

  // Attitude controller
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

void QuadrotorModel::setAttitude(const double roll,
                                 const double pitch,
                                 const double yaw,
                                 const double z) {
  this->attitude_setpoints = Vec4{roll, pitch, yaw, z};
}

void QuadrotorModel::setPosition(const Vec3 &p_G) {
  this->position_setpoints = p_G;
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
