/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP
#define GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP

#include "gvio/util/util.hpp"
#include "gvio/gimbal/calibration/chessboard.hpp"
#include "gvio/camera/distortion.hpp"

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
};

/**
 * CameraProperty to string
 */
std::ostream &operator<<(std::ostream &os, const CameraProperty &cam);

/**
 * Calibration validator
 */
struct CalibValidator {
  std::vector<CameraProperty> cam;
  Chessboard chessboard;

  CalibValidator();
  virtual ~CalibValidator();

  /**
   * Form camera intrinsics matrix K
   *
   * @param cam_id Camera ID
   * @returns Camera intrinsics matrix K
   */
  Mat3 K(const int cam_id);

  /**
   * Form distortion vector D
   *
   * @param cam_id Camera ID
   * @returns Distortion coefficients D
   */
  VecX D(const int cam_id);

  /**
   * Load initial optimization params
   *
   * @param nb_cameras Number of cameras
   * @param calib_file Path to calib file
   * @param target_file Path to target file
   * @returns 0 for success, -1 for failure
   */
  int load(const int nb_cameras,
           const std::string &calib_file,
           const std::string &target_file);

  /**
   * Validate calibration
   *
   * @param cam_id Camera ID
   * @param image Input image
   * @returns Validation image for visual inspection
   */
  cv::Mat validate(const int cam_id, const cv::Mat &image);
};

/**
 * CalibValidator to string
 */
std::ostream &operator<<(std::ostream &os, const CalibValidator &validator);

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP
