/**
 * @file
 * @ingroup quadrotor
 */
#ifndef GVIO_QUADROTOR_QUADROTOR_MODEL_HPP
#define GVIO_QUADROTOR_QUADROTOR_MODEL_HPP

#include <float.h>
#include <iostream>

#include "gvio/util/util.hpp"
#include "gvio/quadrotor/control/attitude_controller.hpp"
#include "gvio/quadrotor/control/position_controller.hpp"
#include "gvio/quadrotor/control/waypoint_controller.hpp"

namespace gvio {
/**
 * @addtogroup quadrotor
 * @{
 */

/**
 * Quadrotor model
 */
class QuadrotorModel {
public:
  Vec3 rpy_G{0.0, 0.0, 0.0}; ///< Attitude in global frame
  Vec3 w_G{0.0, 0.0, 0.0};   ///< Angular velocity in global frame
  Vec3 p_G{0.0, 0.0, 0.0};   ///< Position in global frame
  Vec3 a_G{0.0, 0.0, 0.0};   ///< Acceleration in global frame
  Vec3 v_G{0.0, 0.0, 0.0};   ///< Linear velocity in global frame

  Vec3 w_B{0.0, 0.0, 0.0}; ///< Angular velocity in body frame
  Vec3 a_B{0.0, 0.0, 0.0}; ///< Acceleration in body frame

  double Ix = 0.0963; ///< Moment of inertia in x-axis
  double Iy = 0.0963; ///< Moment of inertia in y-axis
  double Iz = 0.1927; ///< Moment of inertia in z-axis

  double kr = 0.1; ///< Rotation drag constant
  double kt = 0.2; ///< Translation drag constant

  double l = 0.9; ///< Quadrotor arm length
  double d = 1.0; ///< drag constant

  double m = 1.0;  ///< Mass
  double g = 10.0; ///< Gravity

  std::string ctrl_mode = "POS_CTRL_MODE";
  Vec4 attitude_setpoints{0.0, 0.0, 0.0, 0.5}; ///< Attitude setpoints
  Vec3 position_setpoints{0.0, 0.0, 0.0};      ///< Position setpoints
  Mission mission;                             ///< Mission
  AttitudeController attitude_controller;      ///< Attitude controller
  PositionController position_controller;      ///< Position controller
  WaypointController waypoint_controller;      ///< Waypoint controller

  QuadrotorModel() {}
  QuadrotorModel(const Vec3 &rpy_G, const Vec3 &p_G) : rpy_G{rpy_G}, p_G{p_G} {}

  /**
   * Load mission
   *
   * @param mission_file Mission file
   * @returns 0 for success, -1 for failure
   */
  int loadMission(const std::string &mission_file);

  /**
   * Update
   *
   * @param motor_inputs Motor inputs (m1, m2, m3, m4)
   * @param dt Time difference (s)
   * @returns 0 for success, -1 for failure
   */
  int update(const VecX &motor_inputs, const double dt);

  /**
   * Update
   *
   * @param dt Time difference (s)
   * @returns 0 for success, -1 for failure
   */
  int update(const double dt);

  /**
   * Update attitude controller
   *
   * @param dt Time difference (s)
   * @returns Motor command (m1, m2, m3, m4)
   */
  Vec4 attitudeControllerControl(const double dt);

  /**
   * Update position controller
   *
   * @param dt Time difference (s)
   * @returns Attitude command (roll, pitch, yaw, thrust)
   */
  Vec4 positionControllerControl(const double dt);

  /**
   * Update waypoint controller
   *
   * @param dt Time difference (s)
   * @returns Attitude command (roll, pitch, yaw, thrust)
   */
  Vec4 waypointControllerControl(const double dt);

  /**
   * Set quadrotor attitude controller setpoints
   *
   * @param roll Roll
   * @param pitch Pitch
   * @param yaw Yaw
   * @param z Thrust
   */
  void setAttitude(const double roll,
                   const double pitch,
                   const double yaw,
                   const double z);

  /**
   * Set quadrotor position controller setpoints
   *
   * @param p_G Position in global frame
   */
  void setPosition(const Vec3 &p_G);

  /**
   * Print state
   */
  void printState();
};

/** @} group quadrotor */
} // namespace gvio
#endif // GVIO_QUADROTOR_QUADROTOR_MODEL_HPP
