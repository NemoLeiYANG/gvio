/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_KLT_TRACKER_HPP
#define GVIO_FEATURE2D_KLT_TRACKER_HPP

#include "gvio/feature2d/feature_tracker.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * KLT based feature tracker
 */
class KLTTracker : public FeatureTracker {
public:
  KLTTracker() {}

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
  virtual int detect(const cv::Mat &image, Features &features) override;

  /**
   * Track features
   *
   * The idea is that with the current features, we want to match it against
   * the current list of FeatureTrack.
   *
   * @param features List of features in current frame
   * @returns 0 for success, -1 for failure
   */
  int track(const Features &features);

  /**
   * Update feature tracker
   *
   * @param img_cur Current image frame
   * @returns 0 for success, -1 for failure
   */
  virtual int update(const cv::Mat &img_cur) override;
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_KLT_TRACKER_HPP
