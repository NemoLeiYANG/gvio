/**
 * @file
 * @ingroup calibration
 */
#ifndef GVIO_CALIBRATION_CAMERA_PROPERTY_HPP
#define GVIO_CALIBRATION_CAMERA_PROPERTY_HPP

#include "gvio/util/util.hpp"
#include "gvio/camera/distortion.hpp"
#include "gvio/camera/pinhole_model.hpp"

namespace gvio {
/**
 * @addtogroup calibration
 * @{
 */

struct CameraProperty {
  int camera_index;
  std::string camera_model;
  std::string distortion_model;
  VecX distortion_coeffs;
  VecX intrinsics;
  Vec2 resolution;

  CameraProperty();

  // Pinhole model w/o distortion constructor
  CameraProperty(const int camera_index,
                 const double fx,
                 const double fy,
                 const double cx,
                 const double cy,
                 const int image_width,
                 const int image_height);

  // Pinhole model w/o distortion constructor
  CameraProperty(const int camera_index, const Mat3 &K, const Vec2 &resolution);

  // Camera model and distortion model constructor
  CameraProperty(const int camera_index,
                 const std::string &camera_model,
                 const Mat3 &K,
                 const std::string &distortion_model,
                 const VecX &D,
                 const Vec2 &resolution);

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
   * @param image_points Image points [px]
   * @param rect_mat Rectification matrix
   * @returns Undistorted image points [ideal]
   */
  std::vector<cv::Point2f>
  undistortPoints(const std::vector<cv::Point2f> &image_points,
                  const Mat3 &rect_mat = I(3));

  /**
   * Undistort point
   *
   * @param image_point Image point [px]
   * @param rect_mat Rectification matrix
   * @returns Undistorted image point [ideal]
   */
  cv::Point2f undistortPoint(const cv::Point2f &image_point,
                             const Mat3 &rect_mat = I(3));

  /**
   * Distort points
   *
   * @param image_points Image points [ideal]
   * @returns Distorted image points [px]
   */
  std::vector<cv::Point2f>
  distortPoints(const std::vector<cv::Point2f> &points);

  /**
   * Undistort image
   *
   * @param image Image
   * @param balance Balance (between 0.0 and 1.0)
   * @param K_ud Camera intrinsic matrix for undistorted image
   *
   * @returns Undistorted image
   */
  cv::Mat undistortImage(const cv::Mat &image,
                         const double balance,
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
  cv::Mat undistortImage(const cv::Mat &image, const double balance);

  /**
   * Project 3D points to image plane
   *
   * @param X 3D points
   * @returns pixels Pixel point in image plane [px]
   */
  MatX project(const MatX &X);

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point [ideal]
   * @param pixel Point in image plane [px]
   * @return 0 for success, -1 for failure
   */
  Vec2 project(const Vec3 &X);
};

/**
 * CameraProperty to string
 */
std::ostream &operator<<(std::ostream &os, const CameraProperty &cam);

/** @} group calibration */
} // namespace gvio
#endif // GVIO_CALIBRATION_CAMERA_PROPERTY_HPP
