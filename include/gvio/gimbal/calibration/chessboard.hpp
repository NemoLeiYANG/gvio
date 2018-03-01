/**
 * @file
 * @ingroup gimbal
 */
#ifndef GVIO_GIMBAL_CALIBRATION_CHESSBOARD_HPP
#define GVIO_GIMBAL_CALIBRATION_CHESSBOARD_HPP

#include <opencv2/calib3d/calib3d.hpp>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup gimbal
 * @{
 */

/**
 * Chessboard
 */
struct Chessboard {
  int nb_rows = 10;
  int nb_cols = 10;
  double square_size = 0.1;
  std::vector<cv::Point3f> object_points;

  Chessboard();
  virtual ~Chessboard();

  /**
   * Load config file
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &config_file);

  /**
   * Create 2D grid points
   * @returns Vector of gridpoints
   */
  std::vector<Vec2> createGridPoints2d();

  /**
   * Create object points
   * @returns Vector of object points
   */
  std::vector<cv::Point3f> createObjectPoints();

  /**
   * Detect chessboard corners
   *
   * @param image Input image
   * @returns Vector of chessboard corners
   */
  std::vector<cv::Point2f> detect(const cv::Mat &image);

  /**
   * Draw chessboard corners
   * @param image Image where chessboard corners are drawn
   * @returns 0 for success, -1 for failure
   */
  int drawCorners(cv::Mat &image);

  /**
   * Solve PnP between camera and chessboard
   *
   * @param corners Detected chessboard corners
   * @param K Camera intrinsics matrix K
   * @param D Camera distortion coefficients D
   *
   * @returns 0 for success, -1 for failure
   */
  int solvePnP(const std::vector<cv::Point2f> corners,
               const cv::Mat &K,
               const cv::Mat &D,
               Mat4 &T_c_t);

  /**
   * Calculate corner positions
   *
   * @param corners Detected chessboard corners
   * @param K Camera intrinsics matrix K
   * @param D Camera distortion coefficients D
   * @param X Corner positions relative to camera in ideal coordinates
   *
   * @returns 0 for success, -1 for failure
   */
  int calcCornerPositions(const std::vector<cv::Point2f> corners,
                          const cv::Mat &K,
                          const cv::Mat &D,
                          MatX &X);
};

/** @} group gimbal */
} // namespace gvio
#endif // GVIO_GIMBAL_CALIBRATION_CHESSBOARD_HPP
