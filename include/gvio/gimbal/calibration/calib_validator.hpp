/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP
#define GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

/**
 * Calibration validator
 */
struct CalibValidator {
  std::string camera_model;
  std::string distortion_model;
  VecX distortion_coeffs;
  VecX intrinsics;
  Vec2 resolution;

  CalibValidator();
  virtual ~CalibValidator();

  /**
   * Load initial optimization params
   *
   * @param calib_file Path to calib file
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &calib_file);

  /**
   * Validate calibration
   * @param image Input image
   * @returns Validation image for visual inspection
   */
  cv::Mat validate(const cv::Mat &image);
};

/**
 * CalibValidator to string
 */
std::ostream &operator<<(std::ostream &os, const CalibValidator &validator);

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP
