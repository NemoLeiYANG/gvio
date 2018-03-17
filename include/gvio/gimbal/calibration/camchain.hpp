/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CAMCHAIN_HPP
#define GVIO_GIMBAL_CALIBRATION_CAMCHAIN_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

struct CameraProperty {
  std::string camera_model;
  std::string distortion_model;
  VecX distortion_coeffs;
  VecX intrinsics;
  Vec2 resolution;

  Mat3 K();
  VecX D();
};

/**
 * CameraProperty to string
 */
std::ostream &operator<<(std::ostream &os, const CameraProperty &cam);

/**
 * Camchain
 */
class Camchain {
public:
  std::vector<CameraProperty> cam;
  Mat4 T_C1_C0;

  VecX tau_s = zeros(6, 1);
  VecX tau_d = zeros(6, 1);
  VecX w1 = zeros(3, 1);
  VecX w2 = zeros(3, 1);
  double theta1_offset = 0.0;
  double theta2_offset = 0.0;

  Camchain();
  virtual ~Camchain();

  /**
   * Load
   *
   * @param nb_cameras Number of cameras
   * @param camchain_file Path to camchain file
   *
   * @returns 0 for success, -1 for failure
   */
  int load(const int nb_cameras, const std::string &camchain_file);
};

/**
 * Camchain to string
 */
std::ostream &operator<<(std::ostream &os, const Camchain &camchain);

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CAMCHAIN_HPP
