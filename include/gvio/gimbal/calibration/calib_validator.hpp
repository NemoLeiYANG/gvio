/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP
#define GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP

#include "gvio/util/util.hpp"
#include "gvio/camera/distortion.hpp"
#include "gvio/gimbal/gimbal_model.hpp"
#include "gvio/gimbal/calibration/chessboard.hpp"

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
  GimbalModel gimbal_model;
  Mat4 T_C1_C0;

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
   *
   * @returns 0 for success, -1 for failure
   */
  int load(const int nb_cameras,
           const std::string &calib_file,
           const std::string &target_file);

  /**
   * Detect chessboard corners
   *
   * @param image Input image
   * @param K Camera intrinsics matrix K
   * @param D Distortion coefficients vector D
   * @param image_ud Output undistorted image
   * @param Knew Output new camera intrinsics matrix K
   * @param X 3D position of chessboard corners
   *
   * @returns 0 for success, -1 for failure
   */
  int detect(const cv::Mat &image,
             const Mat3 &K,
             const VecX &D,
             cv::Mat &image_ud,
             cv::Mat &Knew,
             MatX &X);

  /**
   * Project 3D points to image plane and draw chessboard corners
   *
   * @param image Input image
   * @param K Camera intrinsics matrix K
   * @param X 3D position of chessboard corners
   * @param color Color to visualize chessboard corners
   *
   * @returns Image with chessboard corners visualized
   */
  cv::Mat projectAndDraw(const cv::Mat &image,
                         const Mat3 &K,
                         const MatX &X,
                         const cv::Scalar &color = cv::Scalar(0, 0, 255));

  /**
   * Project 3D points to image plane and draw chessboard corners
   *
   * @param image Input image
   * @param K Camera intrinsics matrix K
   * @param D Distortion coefficients vector D
   * @param X 3D position of chessboard corners
   * @param color Color to visualize chessboard corners
   *
   * @returns Image with chessboard corners visualized
   */
  cv::Mat projectAndDraw(const cv::Mat &image,
                         const Mat3 &K,
                         const VecX &D,
                         const MatX &X,
                         const cv::Scalar &color = cv::Scalar(0, 0, 255));

  /**
   * Validate calibration
   *
   * @param cam_id Camera ID
   * @param image Input image
   * @returns Validation image for visual inspection
   */
  cv::Mat validate(const int cam_id, const cv::Mat &image);

  /**
   * Validate stereo calibration
   *
   * @param img0 Input image from cam0
   * @param img1 Input image from cam1
   * @returns Validation image for visual inspection
   */
  cv::Mat validateStereo(const cv::Mat &img0, const cv::Mat &img1);

  /**
   * Validate stereo calibration
   *
   * @param img0 Input image from cam0
   * @param img1 Input image from cam1
   * @returns Validation image for visual inspection
   */
  cv::Mat validateStereo2(const cv::Mat &img0, const cv::Mat &img1);

  /**
   * Validate stereo + gimbal calibration
   *
   * @param img0 Input image from cam0
   * @param img1 Input image from cam1
   * @param img2 Input image from cam2
   * @param joint_roll Joint roll angle (radians)
   * @param joint_pitch Joint pitch angle (radians)
   * @returns Validation image for visual inspection
   */
  cv::Mat validateTriclops(const cv::Mat &img0,
                           const cv::Mat &img1,
                           const cv::Mat &img2,
                           const double joint_roll,
                           const double joint_pitch);
};

/**
 * CalibValidator to string
 */
std::ostream &operator<<(std::ostream &os, const CalibValidator &validator);

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CALIB_VALIDATOR_HPP
