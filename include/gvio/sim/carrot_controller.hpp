/**
 * @file
 * @ingroup sim
 */
#ifndef GVIO_SIM_CARROT_CONTROLLER_HPP
#define GVIO_SIM_CARROT_CONTROLLER_HPP

#include <vector>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

class CarrotController {
public:
  std::vector<Vec3> waypoints;
  Vec3 wp_start = Vec3::Zero();
  Vec3 wp_end = Vec3::Zero();
  size_t wp_index = 0;
  double look_ahead_dist = 0.0;

  CarrotController() {}
  virtual ~CarrotController() {}

  /**
   * Configure
   *
   * @param waypoints Waypoints
   * @param look_ahead_dist Look ahead distance (m)
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::vector<Vec3> &waypoints,
                const double look_ahead_dist);

  /**
   * Calculate closest point
   *
   * @param wp_start Start of waypoint
   * @param wp_end End of waypoint
   * @param pos Position of robot
   * @param result Result is a the closest point between `wp_start` and `wp_end`
   *
   * @returns A number to denote progress along the waypoint,
   * - t == -1: Position is before `wp_start`
   * - t == 0: Position is between `wp_start` and `wp_end`
   * - t == 1: Position is after `wp_end`
   */
  int closestPoint(const Vec3 &wp_start,
                   const Vec3 &wp_end,
                   const Vec3 &pos,
                   Vec3 &result);

  /**
   * Calculate carrot point
   *
   * @param wp_start Start of waypoint
   * @param wp_end End of waypoint
   * @param look_ahead_dist Look ahead distance (m)
   * @param pos Position of robot
   * @param result Result is a the carrot point between `wp_start` and `wp_end`
   *
   * @returns A number to denote progress along the waypoint,
   * - t == -1: Position is before `wp_start`
   * - t == 0: Position is between `wp_start` and `wp_end`
   * - t == 1: Position is after `wp_end`
   */
  int carrotPoint(const Vec3 &wp_start,
                  const Vec3 &wp_end,
                  const double look_ahead_dist,
                  const Vec3 &pos,
                  Vec3 &result);

  /**
   * Update carrot controller
   *
   * @param pos Current position
   * @param carrot_pt Carrot point
   * @returns 0 for success, 1 for all waypoints reached and -1 for failure
   */
  int update(const Vec3 &pos, Vec3 &carrot_pt);
};

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_CARROT_CONTROLLER_HPP
