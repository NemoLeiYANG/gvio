/**
 * @file
 * @ingroup apriltag
 */
#ifndef GVIO_APRILTAG_MICHIGAN_HPP
#define GVIO_APRILTAG_MICHIGAN_HPP

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

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>

#include "gvio/util/util.hpp"
#include "gvio/apriltag/base_detector.hpp"
#include "gvio/apriltag/data.hpp"
#include "gvio/camera/camera.hpp"

namespace gvio {
/**
 * @addtogroup apriltag
 * @{
 */

/**
 * Michigan Apriltag Detector
 */
class MichiganDetector : public BaseDetector {
public:
  apriltag_detector_t *detector = nullptr;
  apriltag_family_t *family = nullptr;

  MichiganDetector() {}
  ~MichiganDetector() {
    // detector
    if (this->detector != nullptr) {
      apriltag_detector_destroy(this->detector);
    }

    // family
    if (this->family != nullptr) {
      tag16h5_destroy(this->family);
    }
  }

  /**
   * Configure
   *
   * @param config_file Path to configuration file (YAML)
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Extract AprilTags
   *
   * @param image
   * @param tags
   * @returns 0 for success else failure
   */
  int extractTags(cv::Mat &image, std::vector<TagPose> &tags);

  /**
   * Obtain pose
   *
   * @param tag Tag detected
   * @param tag_pose Tag Pose
   * @returns 0 for success and -1 for failure
   */
  int obtainPose(apriltag_detection_t *tag, TagPose &tag_pose);
};

/** @} group apriltag */
} // namespace gvio
#endif // GVIO_APRILTAG_MICHIGAN_HPP
