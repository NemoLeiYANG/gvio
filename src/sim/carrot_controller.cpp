#include "gvio/sim/carrot_controller.hpp"

namespace gvio {

int CarrotController::configure(const std::vector<Vec3> &waypoints,
                                const double look_ahead_dist) {
  if (waypoints.size() <= (size_t) 2) {
    LOG_ERROR("Too few waypoints!");
    return -1;
  }

  this->waypoints = waypoints;
  this->wp_start = this->waypoints[0];
  this->wp_end = this->waypoints[1];
  this->wp_index = 1;
  this->look_ahead_dist = look_ahead_dist;

  return 0;
}

int CarrotController::closestPoint(const Vec3 &wp_start,
                                   const Vec3 &wp_end,
                                   const Vec3 &pos,
                                   Vec3 &result) {
  // Calculate closest point
  const Vec3 v1 = pos - wp_start;
  const Vec3 v2 = wp_end - wp_start;
  const double t = v1.dot(v2) / v2.squaredNorm();
  result = wp_start + t * v2;

  return t;
}

int CarrotController::carrotPoint(const Vec3 &wp_start,
                                  const Vec3 &wp_end,
                                  const double look_ahead_dist,
                                  const Vec3 &pos,
                                  Vec3 &result) {
  Vec3 closest_pt;
  int t = this->closestPoint(wp_start, wp_end, pos, closest_pt);

  if (t == -1) {
    // Closest point is before wp_start
    result = wp_start;

  } else if (t == 0) {
    // Closest point is between wp_start wp_end
    const Vec3 u = wp_end - wp_start;
    const Vec3 v = u / u.norm();
    result = closest_pt + look_ahead_dist * v;

  } else if (t == 1) {
    // Closest point is after wp_end
    result = wp_end;
  }

  return t;
}

int CarrotController::update(const Vec3 &pos, Vec3 &carrot_pt) {
  // Calculate new carot point
  int status = this->carrotPoint(this->wp_start,
                                 this->wp_end,
                                 this->look_ahead_dist,
                                 pos,
                                 carrot_pt);

  // Check if there are more waypoints
  if ((this->wp_index + 1) == this->waypoints.size()) {
    return 1;
  }
  // std::cout << "status: " << status << std::endl;
  // std::cout << "wp_end: " << this->wp_end.transpose() << std::endl;

  // Update waypoints
  if (status == 1) {
    this->wp_index++;
    this->wp_start = this->wp_end;
    this->wp_end = this->waypoints[this->wp_index];
  }

  return 0;
}

} // namespace gvio
