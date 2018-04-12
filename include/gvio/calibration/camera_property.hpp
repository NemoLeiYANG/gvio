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

  // Pinhole Model w/o distortion constructor
  CameraProperty(const int camera_index,
                 const double fx,
                 const double fy,
                 const double cx,
                 const double cy,
                 const int image_width,
                 const int image_height);

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
   * Undistort point
   *
   * @param image_point Image point
   * @param image_point_ud Undistorted image point
   * @return 0 for success, -1 for failure
   */
  int undistortPoint(const cv::Point2f &image_point,
                     cv::Point2f &image_point_ud);

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
   * Project 3D points to image plane
   *
   * @param X 3D points
   * @param pixels Points in image plane
   * @return 0 for success, -1 for failure
   */
  int project(const MatX &X, MatX &pixels);

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point
   * @param pixel Point in image plane
   * @return 0 for success, -1 for failure
   */
  int project(const Vec3 &X, Vec2 &pixel);
};

/**
 * CameraProperty to string
 */
std::ostream &operator<<(std::ostream &os, const CameraProperty &cam);

/** @} group calibration */
} // namespace gvio
#endif // GVIO_CALIBRATION_CAMERA_PROPERTY_HPP
