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

class AttitudeController {
public:
  double dt;
  Vec4 outputs;

  PID roll_controller;
  PID pitch_controller;
  PID yaw_controller;

  AttitudeController()
      : dt(0.0), outputs(), roll_controller(200.0, 0.5, 10.0),
        pitch_controller(200.0, 0.5, 10.0), yaw_controller(200.0, 0.5, 10.0) {}

  Vec4 update(const Vec4 &setpoints, const Vec4 &actual, double dt);
  Vec4 update(const Vec4 &psetpoints,
              const Vec4 &vsetpoints,
              const Vec4 &actual,
              double dt);
};

class PositionController {
public:
  double dt;
  Vec4 outputs;

  PID x_controller;
  PID y_controller;
  PID z_controller;

  PositionController()
      : dt(0.0), outputs(), x_controller(0.5, 0.0, 0.035),
        y_controller(0.5, 0.0, 0.035), z_controller(0.5, 0.0, 0.018) {}

  Vec4 update(const Vec3 &setpoints, const Vec4 &actual, double yaw, double dt);
};

class QuadrotorModel {
public:
  Vec3 attitude;         ///< Attitude
  Vec3 angular_velocity; ///< Angular velocity
  Vec3 position;         ///< Position
  Vec3 linear_velocity;  ///< Linear velocity

  double Ix; ///< Moment of inertial in x-axis
  double Iy; ///< Moment of inertial in y-axis
  double Iz; ///< Moment of inertial in z-axis

  double kr; ///< Rotation drag constant
  double kt; ///< Translation drag constant

  double l; ///< Quadrotor arm length
  double d; ///< drag constant

  double m; ///< Mass
  double g; ///< Gravity

  Vec4 attitude_setpoints; ///< Quadrotor attitude setpoints
  Vec3 position_setpoints; ///< Quadrotor position setpoints

  AttitudeController attitude_controller; ///< Quadrotor attitude controller
  PositionController position_controller; ///< Quadrotor position controller

  QuadrotorModel()
      : attitude(0, 0, 0), angular_velocity(0, 0, 0), position(0, 0, 0),
        linear_velocity(0, 0, 0), Ix(0.0963), // Inertial x
        Iy(0.0963),                           // Inertial y
        Iz(0.1927),                           // Inertial z
        kr(0.1),                              // Rotation drag constant
        kt(0.2),                              // Translation drag constant
        l(0.9),                               // Arm length
        d(1.0),                               // Drag
        m(1.0),                               // Mass of quad
        g(10.0),                              // Gravitational constant
        attitude_setpoints(0, 0, 0, 0), position_setpoints(0, 0, 0),
        attitude_controller(), position_controller() {}

  QuadrotorModel(const VecX &pose)
      : attitude(pose(3), pose(4), pose(5)), angular_velocity(0, 0, 0),
        position(pose(0), pose(1), pose(2)), linear_velocity(0, 0, 0),
        Ix(0.0963), // Inertial x
        Iy(0.0963), // Inertial y
        Iz(0.1927), // Inertial z
        kr(0.1),    // Rotation drag constant
        kt(0.2),    // Translation drag constant
        l(0.9),     // Arm length
        d(1.0),     // Drag
        m(1.0),     // Mass of quad
        g(10.0),    // Gravitational constant
        attitude_setpoints(0, 0, 0, 0.5), position_setpoints(0, 0, 0),
        attitude_controller(), position_controller() {}

  /**
   * Update
   *
   * @param motor_inputs Motor inputs (m1, m2, m3, m4)
   * @param dt Time difference (s)
   * @returns 0 for success, -1 for failure
   */
  int update(const VecX &motor_inputs, const double dt);

  /**
   * Update attitude controller
   *
         * @param dt Time difference (s)
         * @returns Attitude command (roll, pitch, yaw, thrust)
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
   * Get velocity
   * @returns Linear Velocity as a vector (vx, vy, vz)
   */
  Vec3 getVelocity();

  /**
   * Get velocity
   * @returns Angular Velocity as a vector (wx, wy, wz)
   */
  Vec3 getAngularVelocity();

  /**
   * Print state
   */
  void printState();
};

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_QUADROTOR_HPP
