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
  Vec3 waypoint = Vec3::Zero();
  int retval = mission.update(p_G, waypoint);
  if (retval != 0) {
    return retval;
  }

  // Calculate waypoint relative to quadrotor
  Vec3 errors = T_P_W{pose.orientation} * Vec3{waypoint - p_G};

  // Calculate velocity relative to quadrotor
  const Vec3 v_B = T_P_W{pose.orientation} * v_G;

  // Roll
  double r = -this->ct_controller.update(errors(1), this->dt);

  // Pitch
  double error_forward = mission.desired_velocity - v_B(0);
  double p = this->at_controller.update(error_forward, this->dt);

  // Yaw
  double y = mission.waypointHeading();

  // Throttle
  const double error_z = waypoint(2) - p_G(2);
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
  this->setpoints = waypoint;
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
