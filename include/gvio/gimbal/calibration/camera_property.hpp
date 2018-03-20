/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CAMERA_PROPERTY_HPP
#define GVIO_GIMBAL_CALIBRATION_CAMERA_PROPERTY_HPP

#include "gvio/util/util.hpp"
#include "gvio/camera/distortion.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

struct CameraProperty {
  int camera_index;
  std::string camera_model;
  std::string distortion_model;
  VecX distortion_coeffs;
  VecX intrinsics;
  Vec2 resolution;

  /**
   * Camera intrinsics matrix K
   * @returns Camera intrinsics matrix K
   */
  Mat3 K();

  /**
   * Distortion coefficients D
   * @returns Distortion coefficients D
   */
  VecX D();

  /**
   * Undistort points
   *
   * @param image_points Image points
   * @param image_points_ud Undistorted image points
   * @return 0 for success, -1 for failure
   */
  int undistortPoints(const std::vector<cv::Point2f> &image_points,
                      std::vector<cv::Point2f> &image_points_ud);

  /**
   * Undistort image
   *
   * @param image Image
   * @param balance Balance (between 0.0 and 1.0)
   * @param image_ud Undistorted image
   * @param K_ud Camera intrinsic matrix for undistorted image
   *
   * @return 0 for success, -1 for failure
   */
  int undistortImage(const cv::Mat &image,
                     const double balance,
                     cv::Mat &image_ud,
                     cv::Mat &K_ud);

  /**
   * Undistort image
   *
   * @param image Image
   * @param balance Balance (between 0.0 and 1.0)
   * @param image_ud Undistorted image
   *
   * @return 0 for success, -1 for failure
   */
  int undistortImage(const cv::Mat &image,
                     const double balance,
                     cv::Mat &image_ud);

  /**
   * Project 3D point to image plane
   *
   * @param X 3D points
   * @param pixels Points in image plane
   * @return 0 for success, -1 for failure
   */
  int project(const MatX &X, MatX &pixels);
};

/**
 * CameraProperty to string
 */
std::ostream &operator<<(std::ostream &os, const CameraProperty &cam);

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CAMERA_PROPERTY_HPP
