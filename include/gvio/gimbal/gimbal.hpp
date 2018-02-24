/**
 * @file
 * @defgroup gimbal gimbal
 */
#ifndef GVIO_GIMBAL_HPP
#define GVIO_GIMBAL_HPP

#include <iostream>
#include <map>
#include <math.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "gvio/util/util.hpp"
#include "gvio/gimbal/sbgc.hpp"
#include "gvio/gimbal/saliency.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

class Gimbal {
public:
  bool configured = false;

  SBGC sbgc;
  double roll_limits[2] = {0.0, 0.0};
  double pitch_limits[2] = {0.0, 0.0};

  Vec3 setpoints = Vec3{0.0, 0.0, 0.0};

  Vec3 imu_accel = Vec3{0.0, 0.0, 0.0};
  Vec3 imu_gyro = Vec3{0.0, 0.0, 0.0};
  Vec3 camera_angles = Vec3{0.0, 0.0, 0.0};
  Vec3 frame_angles = Vec3{0.0, 0.0, 0.0};
  Vec3 encoder_angles = Vec3{0.0, 0.0, 0.0};

  Gimbal();
  virtual ~Gimbal();

  /**
   * Configure
   *
   * @param config_path Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_path);

  /**
   * Turn gimbal on
   * @returns 0 for success, -1 for failure
   */
  int on();

  /**
   * Turn gimbal off
   * @returns 0 for success, -1 for failure
   */
  int off();

  /**
   * Update gimbal states
   * @returns 0 for success, -1 for failure
   */
  int update();

  /**
   * Set gimbal angle
   *
   * @param roll Roll (radians)
   * @param pitch Pitch (radians)
   * @returns 0 for success, -1 for failure
   */
  int setAngle(const double roll, const double pitch);

  /**
   * Print setpoints
   */
  void printSetpoints();
};

} // namespace gvio
/** @} group gimbal */
#endif // GVIO_GIMBAL_HPP
