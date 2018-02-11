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
  bool enable_tracking = false;

  Vec3 setpoints = Vec3{0.0, 0.0, 0.0};
  Vec3 target_P = Vec3{0.0, 0.0, 0.0};

  Vec3 imu_accel = Vec3{0.0, 0.0, 0.0};
  Vec3 imu_gyro = Vec3{0.0, 0.0, 0.0};
  Vec3 camera_angles = Vec3{0.0, 0.0, 0.0};
  Vec3 frame_angles = Vec3{0.0, 0.0, 0.0};
  Vec3 encoder_angles = Vec3{0.0, 0.0, 0.0};

  Gimbal() {}
  ~Gimbal() { this->off(); }

  int configure(const std::string &config_path);
  int on();
  int off();
  int trackTarget(const Vec3 &target_P);
  int updateGimbalStates();
  int setAngle(const double roll, const double pitch);
  void printSetpoints();
};

} // namespace gvio
/** @} group gimbal */
#endif // GVIO_GIMBAL_HPP
