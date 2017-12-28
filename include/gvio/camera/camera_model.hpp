/**
 * @file
 * @ingroup camera
 */
#ifndef GVIO_CAMERA_CAMERA_MODEL_HPP
#define GVIO_CAMERA_CAMERA_MODEL_HPP

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Camera model
 */
class CameraModel {
public:
  CameraModel() {}

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point
   * @param R Rotation matrix
   * @param t translation vector
   *
   * @returns 3D point in image plane (homogenous)
   */
  virtual Vec3 project(const Vec3 &X, const Mat3 &R, const Vec3 &t) = 0;

  /**
   * Convert pixel measurement to image coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to image coordinates
   */
  virtual Vec2 pixel2image(const Vec2 &pixel) = 0;

  /**
   * Convert pixel measurement to image coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to image coordinates
   */
  virtual Vec2 pixel2image(const Vec2 &pixel) const = 0;

  /**
   * Convert pixel measurement to image coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to image coordinates
   */
  virtual Vec2 pixel2image(const cv::Point2f &pixel) = 0;

  /**
   * @copydoc Vec2 pixel2image(const cv::Point2f &pixel)
   */
  virtual Vec2 pixel2image(const cv::Point2f &pixel) const = 0;

  /**
   * Return features are observed by camera
   */
  virtual MatX observedFeatures(const MatX &features,
                                const Vec3 &rpy,
                                const Vec3 &t,
                                std::vector<int> &mask) = 0;
};

/** @} group camera */
} // namespace gvio
#endif // GVIO_CAMERA_CAMERA_MODEL_HPP
