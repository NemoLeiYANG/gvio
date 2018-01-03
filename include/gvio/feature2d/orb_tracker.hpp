/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_ORB_TRACKER_HPP
#define GVIO_FEATURE2D_ORB_TRACKER_HPP

#include "gvio/feature2d/feature_tracker.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * ORB based feature tracker
 */
class ORBTracker : public FeatureTracker {
public:
  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  ORBTracker() {
  }

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Detect features
   *
   * @param image Input image
   * @param features List of features
   * @returns 0 for success, -1 for failure
   */
  int detect(const cv::Mat &image, std::vector<Feature> &features);
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_ORB_TRACKER_HPP
