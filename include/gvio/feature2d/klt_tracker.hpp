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
  int detect(const cv::Mat &image, std::vector<Feature> &features);

  /**
   * Draw tracks
   *
   * @param img_cur Current image frame
   * @param p0 Previous corners
   * @param p1 Current corners
   * @param status Corners status
   * @returns Image with feature matches between previous and current frame
   */
  cv::Mat drawTracks(const cv::Mat &img_cur,
                     const std::vector<cv::Point2f> p0,
                     const std::vector<cv::Point2f> p1,
                     const std::vector<uchar> &status);

  /**
   * Track features
   *
   * The idea is that with the current features, we want to match it against
   * the current list of FeatureTrack.
   *
   * @param features List of features in current frame
   * @returns 0 for success, -1 for failure
   */
  int track(const std::vector<Feature> &features);

  /**
   * Update feature tracker
   *
   * @param img_cur Current image frame
   * @returns 0 for success, -1 for failure
   */
  int update(const cv::Mat &img_cur);
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_KLT_TRACKER_HPP
