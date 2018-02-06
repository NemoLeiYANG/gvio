/**
 * @file
 * @ingroup sim
 */
#ifndef GVIO_SIM_QUADROTOR_HPP
#define GVIO_SIM_QUADROTOR_HPP

#include <float.h>
#include <iostream>

#include "gvio/util/util.hpp"
#include "gvio/sim/pid.hpp"

// #include "atl/data/transform.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Attitude controller
 */
class AttitudeController {
public:
  double dt = 0.0;
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  // PID tunes for dt = 0.001 (i.e. 1000Hz)
  PID roll_controller = PID(200.0, 0.5, 10.0);
  PID pitch_controller = PID(200.0, 0.5, 10.0);
  PID yaw_controller = PID(200.0, 0.5, 10.0);

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  // PID roll_controller = PID(10.0, 0.001, 1.0);
  // PID pitch_controller = PID(10.0, 0.001, 1.0);
  // PID yaw_controller = PID(10.0, 0.001, 1.0);

  AttitudeController() {}

  /**
   * Update
   *
   * @param setpoints Setpoints (roll, pitch, yaw, z)
   * @param actual Actual (roll, pitch, yaw, z)
   * @param dt Time difference (s)
   *
   * @returns Motor command (m1, m2, m3, m4)
   */
  Vec4 update(const Vec4 &setpoints, const Vec4 &actual, const double dt);
};

/**
 * Position controller
 */
class PositionController {
public:
  double dt = 0.0;
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  // PID tunes for dt = 0.001 (i.e. 1000Hz)
  PID x_controller = PID(0.5, 0.0, 0.035);
  PID y_controller = PID(0.5, 0.0, 0.035);
  PID z_controller = PID(0.3, 0.0, 0.1);

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  // PID x_controller = PID(5.0, 0.0, 3.5);
  // PID y_controller = PID(5.0, 0.0, 3.5);
  // PID z_controller = PID(3.0, 0.0, 1.0);

  PositionController() {}

  /**
   * Update
   *
   * @param setpoints Setpoints (x, y, z)
   * @param actual Actual (x, y, z)
   * @param yaw (radians)
   * @param dt Time difference (s)
   *
   * @returns Attitude command (roll, pitch, yaw, thrust)
   */
  Vec4 update(const Vec3 &setpoints,
              const Vec4 &actual,
              const double yaw,
              const double dt);
};

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
  Vec4 attitude_setpoints{0.0, 0.0, 0.0, 0.5}; ///< Quadrotor attitude setpoints
  Vec3 position_setpoints{0.0, 0.0, 0.0};      ///< Quadrotor position setpoints
  AttitudeController attitude_controller; ///< Quadrotor attitude controller
  PositionController position_controller; ///< Quadrotor position controller

  QuadrotorModel() {}
  QuadrotorModel(const Vec3 &rpy_G, const Vec3 &p_G) : rpy_G{rpy_G}, p_G{p_G} {}

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
   * @param x Position in x-axis
   * @param y Position in y-axis
   * @param z Position in z-axis
   */
  void setPosition(const double x, const double y, const double z);

  /**
   * Get pose
   * @returns Pose as a vector (x, y, z, roll, pitch, yaw)
   */
  VecX getPose();

  /**
   * Get velocity in the global frame
   * @returns Linear Velocity as a vector (vx_G, vy_G, vz_G)
   */
  Vec3 getVelocity();

  /**
   * Get angular velocity in the global frame
   * @returns Angular velocity as a vector (wx_G, wy_G, wz_G)
   */
  Vec3 getAngularVelocity();

  /**
   * Get angular velocity in the body frame
   * @returns Body angular velocity as a vector (wx_B, wy_B, wz_B)
   */
  Vec3 getBodyAngularVelocity();

  /**
   * Get acceleration in the body frame
   * @returns Body acceleration as a vector (ax_B, ay_B, az_B)
   */
  Vec3 getBodyAcceleration();

  /**
   * Print state
   */
  void printState();
};

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_QUADROTOR_HPP
