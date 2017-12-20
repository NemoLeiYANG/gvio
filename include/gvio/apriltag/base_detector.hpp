/**
 * @file
 * @ingroup apriltag
 */
#ifndef GVIO_APRILTAG_BASE_HPP
#define GVIO_APRILTAG_BASE_HPP

#include <cmath>
#include <fstream>
#include <iostream>
#include <libgen.h>
#include <math.h>
#include <sys/time.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gvio/util/util.hpp"
#include "gvio/camera/camera.hpp"

namespace gvio {
/**
 * @addtogroup apriltag
 * @{
 */

class TagPose {
public:
  int id;
  bool detected;
  Vec3 position;
  Quaternion orientation;

  TagPose() {
    this->id = -1;
    this->detected = false;
    this->position << 0.0, 0.0, 0.0;
    this->orientation = Quaternion();
  };

  TagPose(int id, bool detected, Vec3 position, Quaternion orientation) {
    this->id = id;
    this->detected = detected;
    this->position = position;
    this->orientation = orientation;
  }

  TagPose(int id, bool detected, Vec3 position, Mat3 rotmat) {
    this->id = id;
    this->detected = detected;
    this->position = position;
    this->orientation = Quaternion(rotmat);
  }

  void print() {
    std::cout << "tag ";
    std::cout << "id: " << this->id << "\t";
    std::cout << "detected: " << this->detected << "\t";
    std::cout << "position: ";
    std::cout << "(";
    std::cout << this->position(0) << ", ";
    std::cout << this->position(1) << ", ";
    std::cout << this->position(2);
    std::cout << ")";
    std::cout << std::endl;
  }
};

class BaseDetector {
public:
  bool configured = false;

  TagPose prev_tag;
  int prev_tag_image_width = 0;
  int prev_tag_image_height = 0;

  bool image_cropped = false;
  int crop_x = 0;
  int crop_y = 0;
  int crop_width = 0;
  int crop_height = 0;

  std::map<int, float> tag_configs = {};
  double tag_sanity_check = FLT_MAX;

  std::string camera_mode;
  std::vector<std::string> camera_modes;
  std::map<std::string, CameraConfig> camera_configs;

  bool illum_invar = false;
  bool windowing = false;
  double window_padding = FLT_MAX;
  bool imshow = false;

  BaseDetector() {}
  ~BaseDetector() {}

  /**
   * Configure
   *
   * @param config_file Path to configuration file (YAML)
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Illumination invariant transform
   *
   * @param image Image to be transformed
   * @returns 0 for success, -1 for failure
   */
  int illuminationInvariantTransform(cv::Mat &image);

  /**
   * Change mode
   *
   * @param image Image
   * @returns
   *    - 0: Do not change mode
   *    - 1: Change mode
   */
  int changeMode(const cv::Mat &image);

  /**
   * Get camera intrinsics
   *
   * @param fx Focal-length in x axis
   * @param fy Focal-length in y axis
   * @param px Principle center in x axis
   * @param py Principle center in y axis
   * @param image_width Image width
   * @param image_height Image height
   *
   * @returns
   *    - 0: Success
   *    - -1: Failure
   */
  int getCameraIntrinsics(double *fx,
                          double *fy,
                          double *px,
                          double *py,
                          double *image_width,
                          double *image_height);

  /**
   * Calculate tag corners in inertial frame
   *
   * @param image Image
   * @param tag_pose Tag pose
   * @param padding Corner padding
   * @param top_left Top left tag corner
   * @param btm_right Bottom right tag corner
   *
   * @returns
   *    - 0: Success
   *    - -1: Failure
   */
  int calculateTagCorners(const cv::Mat &image,
                          const TagPose &tag_pose,
                          const double padding,
                          Vec2 &top_left,
                          Vec2 &btm_right);

  /**
   * Get tag size
   *
   * @param tag_pose Tag pose
   * @param tag_size Tag size
   *
   * @returns
   *    - 0: Success
   *    - -1: Failure
   */
  int getTagSize(const TagPose &tag_pose, double *tag_size);

  /**
   * Mask image
   *
   * @param prev_tag Previous AprilTag pose
   * @param image Image
   * @param padding Mask padding
   *
   * @returns:
   *    - 0: Success
   *    - -1: Current image dimensions != previous image dimensions
   *    - -2: AprilTag ID is not in list of IDs to look out for
   *    - -3: Image dimension / configuration mismatch
   */
  int maskImage(const TagPose &prev_tag,
                const cv::Mat &image,
                const double padding = 0.5);

  /**
   * Crop image
   *
   * @param tag_pose AprilTag pose
   * @param image Image
   * @param cropped_image Cropped image
   * @param padding Crop padding
   *
   * @returns:
   *    - 0: Success
   *    - -1: Current image dimensions != previous image dimensions
   *    - -2: AprilTag ID is not in list of IDs to look out for
   *    - -3: Image dimension / configuration mismatch
   */
  int cropImage(const TagPose &tag_pose,
                const cv::Mat &image,
                cv::Mat &cropped_image,
                const double padding = 0.5);

  /**
   * Get relative transform
   *
   * @param p1 AprilTag image point 1
   * @param p2 AprilTag image point 2
   * @param p3 AprilTag image point 3
   * @param p4 AprilTag image point 4
   * @param tag_pose Relative tag pose
   *
   * @returns
   *    - 0: Success
   *    - -1: Failure
   */
  int getRelativePose(const Vec2 &p1,
                      const Vec2 &p2,
                      const Vec2 &p3,
                      const Vec2 &p4,
                      TagPose &tag_pose);

  /**
   * Print detected AprilTag
   *
   * @param tag AprilTag pose
   */
  void printTag(const TagPose &tag);
};

/** @} group apriltag */
} // namespace gvio
#endif // GVIO_APRILTAG_BASE_HPP
