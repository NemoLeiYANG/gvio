#ifndef GVIO_GIMBAL_CALIBRATION_CALIB_PREPROCESSOR_HPP
#define GVIO_GIMBAL_CALIBRATION_CALIB_PREPROCESSOR_HPP

#include <string>

#include <opencv2/calib3d/calib3d.hpp>

#include "gvio/util/util.hpp"
#include "gvio/gimbal/calibration/aprilgrid.hpp"

namespace gvio {

struct CameraProperties {
  int camera_id = 0;            ///< Camera id
  std::string camera_model;     ///< Camera model
  Vec4 intrinsics;              ///< Camera intrinsics (fx, fy, cx, fy)
  std::string distortion_model; ///< Distortion model
  VecX distortion_coeffs;       ///< Distortion coefficients (k1, k2, k3, k4)
  Vec2 resolution;              ///< Resolution

  /// Form camera intrinsics matrix K
  cv::Mat K();

  /// Form distortion coefficients vector D
  cv::Mat D();
};

std::ostream &operator<<(std::ostream &os, const CameraProperties &property);

struct CalibTarget {
  std::string type;
  int rows = 0;
  int cols = 0;
  double square_size = 0.0;
  double spacing = 0.0;
};

std::ostream &operator<<(std::ostream &os, const CalibTarget &target);

class CalibPreprocessor {
public:
  CalibTarget target;
  std::vector<CameraProperties> camera_properties;
  int nb_measurements = 0;
  MatX joint_data;

  CalibPreprocessor();
  virtual ~CalibPreprocessor();

  /**
   * Load target file
   *
   * @param target_file Path to target file
   * @returns 0 for success, -1 for failure
   */
  int loadTargetFile(const std::string &target_file);

  /**
   * Load joint file
   *
   * @param joint_file Path to joint file
   * @returns 0 for success, -1 for failure
   */
  int loadJointFile(const std::string &joint_file);

  /**
   * Load camchain file
   *
   * @param camchain_file Path to camchain file
   * @returns 0 for success, -1 for failure
   */
  int loadCamchainFile(const std::string &camcahin_file);

  /**
   * Find image files
   *
   * @param search_path Path to image files
   * @param image_files List of sorted image files
   * @returns 0 for success, -1 for failure
   */
  int findImageFiles(const std::string &search_path,
                     std::vector<std::string> &image_files);

  /**
   * Undistort image
   *
   * @param K Camera intrinsics matrix K
   * @param D Distortion coefficients D
   * @param image Input image
   * @param Knew New camera matrix K for undistorted image
   *
   * @returns Undistorted image
   */
  cv::Mat undistortImage(const cv::Mat &K,
                         const cv::Mat &D,
                         const cv::Mat &image,
                         cv::Mat &Knew);

  /**
   * Undistort image
   *
   * @param K Camera intrinsics matrix K
   * @param D Distortion coefficients D
   * @param image Input image
   *
   * @returns Undistorted image
   */
  cv::Mat undistortImage(const cv::Mat &K,
                         const cv::Mat &D,
                         const cv::Mat &image);

  /**
   * Find common tags between detected
   *
   * @param tags0 Tags detected from image0
   * @param tags1 Tags detected from image1
   * @param tags2 Tags detected from image2
   *
   * @returns List of common tag ids detected
   */
  std::vector<int>
  findCommonTags(const std::map<int, std::vector<cv::Point2f>> &tags0,
                 const std::map<int, std::vector<cv::Point2f>> &tags1,
                 const std::map<int, std::vector<cv::Point2f>> &tags2);

  /**
   * Preprocess calibration data
   *
   * @param dir_path Path to calibration data
   * @returns 0 for success, -1 for failure
   */
  int preprocess(const std::string &dir_path);
};

} // namespace gvio
#endif
