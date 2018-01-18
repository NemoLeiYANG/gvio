/**
 * @file
 * @ingroup camera
 */
#ifndef GVIO_CAMERA_PINHOLE_MODEL_HPP
#define GVIO_CAMERA_PINHOLE_MODEL_HPP

#include "gvio/util/util.hpp"
#include "gvio/camera/camera_model.hpp"

namespace gvio {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Pinhole camera model
 */
class PinholeModel : public CameraModel {
public:
  std::string model_name = "pinhole";

  MatX K = zeros(3, 3); ///< Camera intrinsics
  double cx = 0.0;      ///< Principle center in x-axis
  double cy = 0.0;      ///< Principle center in y-axis
  double fx = 0.0;      ///< Focal length in x-axis
  double fy = 0.0;      ///< Focal length in y-axis

  PinholeModel() {}
  virtual ~PinholeModel() {}

  PinholeModel(const int image_width, const int image_height, const MatX &K)
      : CameraModel{image_width, image_height}, K{K}, cx{K(0, 2)}, cy{K(1, 2)},
        fx{K(0, 0)}, fy{K(1, 1)} {}

  PinholeModel(const int image_width,
               const int image_height,
               const double fx,
               const double fy,
               const double cx,
               const double cy)
      : CameraModel{image_width, image_height}, cx{cx}, cy{cy}, fx{fx}, fy{fy} {
    this->K = Mat3::Zero();
    K(0, 0) = fx;
    K(1, 1) = fy;
    K(0, 2) = cx;
    K(1, 2) = cy;
    K(2, 2) = 1.0;
  }

  /**
   * Configure
   *
   * @param config_file Path to configuration file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Focal length in x-axis
   *
   * @param image_width Image width in pixels
   * @param fov Field of view in degrees
   * @returns Theoretical focal length in x-axis
   */
  static double focalLengthX(const int image_width, const double fov);

  /**
   * Focal length in y-axis
   *
   * @param image_height Image height in pixels
   * @param fov Field of view in degrees
   * @returns Theoretical focal length in y-axis
   */
  static double focalLengthY(const int image_height, const double fov);

  /**
   * Focal length
   *
   * @param image_width Image width in pixels
   * @param image_height Image height in pixels
   * @param fov Field of view in degrees
   * @returns Theoretical focal length
   */
  static Vec2 focalLength(const int image_width,
                          const int image_height,
                          const double fov);

  /**
   * Return projection matrix
   *
   * @param R Rotation matrix
   * @param t translation vector
   *
   * @returns Projection matrix
   */
  Mat34 P(const Mat3 &R, const Vec3 &t);

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point
   * @param R Rotation matrix
   * @param t translation vector
   *
   * @returns 3D point in image plane (homogenous)
   */
  Vec2 project(const Vec3 &X, const Mat3 &R, const Vec3 &t) override;

  /**
   * Project 3D point to image plane
   *
   * @param X 3D point in homogeneous coordinates
   * @param R Rotation matrix
   * @param t translation vector
   *
   * @returns 3D point in image plane (homogenous)
   */
  Vec3 project(const Vec4 &X, const Mat3 &R, const Vec3 &t) override;

  /**
   * Convert pixel measurement to image coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to image coordinates
   */
  Vec2 pixel2image(const Vec2 &pixel) override;

  /**
   * Convert pixel measurement to image coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to image coordinates
   */
  Vec2 pixel2image(const cv::Point2f &pixel) override;

  /**
   * Convert pixel measurement to image coordinates
   *
   * @param pixel Pixel measurement
   * @returns Pixel measurement to image coordinates
   */
  Vec2 pixel2image(const cv::KeyPoint &pixel) override;

  /**
   * @copydoc Vec2 pixel2image(const Vec2 &pixel)
   */
  Vec2 pixel2image(const Vec2 &pixel) const override;

  /**
   * @copydoc Vec2 pixel2image(const cv::Point2f &pixel)
   */
  Vec2 pixel2image(const cv::Point2f &pixel) const override;

  /**
   * @copydoc Vec2 pixel2image(const cv::KeyPoint &pixel)
   */
  Vec2 pixel2image(const cv::KeyPoint &pixel) const override;
};

/** @} group camera */
} // namespace gvio
#endif // GVIO_CAMERA_PINHOLE_MODEL_HPP
