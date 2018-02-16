#include "gvio/quadrotor/control/waypoint_controller.hpp"

namespace gvio {

double WaypointController::calcYawToWaypoint(const Vec3 &waypoint,
                                             const Vec3 &position) {
  // Assume waypoints are in NWU inertial frame
  const double dx = waypoint(0) - position(0);
  const double dy = waypoint(1) - position(1);

  // Calculate heading
  double heading = atan2(dy, dx);
  if (heading > M_PI) {
    heading -= 2 * M_PI;
  } else if (heading < -M_PI) {
    heading += 2 * M_PI;
  }

  return heading;
}

int WaypointController::update(Mission &mission,
                               const Vec3 &p_G,
                               const Vec3 &rpy_G,
                               const Vec3 &v_G,
                               const double dt) {
  // Check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return 0;
  }

  // Current waypoint
  Vec3 wp_G = Vec3::Zero();
  int retval = mission.update(p_G, wp_G);
  if (retval != 0) {
    return retval;
  }

  // Calculate waypoint relative to quadrotor
  Mat4 T_P_W = zeros(4, 4);
  T_P_W.block(0, 0, 3, 3) = euler123ToRot(yaw(rpy_G));
  const Vec4 wp_B_homo{wp_G(0) - p_G(0),
                       wp_G(1) - p_G(1),
                       wp_G(2) - p_G(2),
                       1.0};
  const Vec4 errors = T_P_W * wp_B_homo;

  // Calculate velocity relative to quadrotor
  const Vec4 v_G_homo{v_G(0), v_G(1), v_G(2), 1.0};
  const Vec4 v_B = T_P_W * v_G_homo;

  // Roll
  double r = -this->ct_controller.update(errors(1), this->dt);

  // Pitch
  double error_forward = mission.desired_velocity - v_B(0);
  double p = this->at_controller.update(error_forward, this->dt);

  // Yaw
  double y = mission.waypointHeading();

  // Throttle
  const double error_z = wp_G(2) - p_G(2);
  double t = this->hover_throttle;
  t += this->z_controller.update(error_z, this->dt);
  t /= fabs(cos(r) * cos(p)); // adjust throttle for roll and pitch

  // Limit roll, pitch and throttle
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;
  t = (t < 0.0) ? 0.0 : t;
  t = (t > 1.0) ? 1.0 : t;

  // Keep track of setpoints and outputs
  this->setpoints = wp_G;
  this->outputs << r, p, y, t;
  this->dt = 0.0;

  return 0;
}

void WaypointController::reset() {
  this->at_controller.reset();
  this->ct_controller.reset();
  this->z_controller.reset();
  this->yaw_controller.reset();
}

} // namespace gvio
