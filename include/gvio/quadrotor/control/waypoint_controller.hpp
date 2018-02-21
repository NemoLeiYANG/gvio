/**
 * @file
 * @ingroup quadrotor
 */
#ifndef GVIO_QUADROTOR_CONTROL_WAYPOINT_CONTROLLER_HPP
#define GVIO_QUADROTOR_CONTROL_WAYPOINT_CONTROLLER_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <vector>

#include "gvio/util/util.hpp"
#include "gvio/control/pid.hpp"
#include "gvio/quadrotor/mission.hpp"

namespace gvio {
/**
 * @addtogroup quadrotor
 * @{
 */

/**
 * Waypoint Controller
 */
class WaypointController {
public:
  bool configured = false;

  double dt = 0.0;

  PID at_controller = PID(0.5, 0.0, 0.035);
  PID ct_controller = PID(0.5, 0.0, 0.035);
  PID z_controller = PID(0.3, 0.0, 0.1);
  PID yaw_controller = PID(2.0, 0.0, 0.1);

  double roll_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
  double pitch_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
  double hover_throttle = 0.5;

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  WaypointController() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return
   *    - 0: Success
   *    - -1: Failed to load config file
   *    - -2: Failed to load mission file
   */
  int configure(const std::string &config_file);

  /**
   * Update controller
   *
   * @param mission Mission
   * @param p_G Actual position in global frame
   * @param v_G Actual velocity in global frame
   * @param rpy_G Actual roll, pitch and yaw in global frame
   * @param dt Time difference in seconds
   *
   * @return
   *   - 0: Success
   *   - -1: Not configured
   *   - -2: No more waypoints
   */
  int update(Mission &mission,
             const Vec3 &p_G,
             const Vec3 &v_G,
             const Vec3 &rpy_G,
             const double dt);

  /**
   * Reset controller errors to 0
   */
  void reset();
};

/** @} group quadrotor */
} // namespace gvio
#endif // GVIO_QUADROTOR_CONTROL_WAYPOINT_CONTROLLER_HPP
