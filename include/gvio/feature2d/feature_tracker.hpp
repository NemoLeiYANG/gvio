/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_FEATURE_TRACKER_HPP
#define GVIO_FEATURE2D_FEATURE_TRACKER_HPP

#include <vector>
#include <map>
#include <algorithm>

#include "gvio/util/util.hpp"
#include "gvio/calibration/calibration.hpp"
#include "gvio/feature2d/draw.hpp"
#include "gvio/feature2d/feature.hpp"
#include "gvio/feature2d/feature_track.hpp"
#include "gvio/feature2d/feature_container.hpp"
#include "gvio/feature2d/gms_matcher.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Feature tracker
 */
class FeatureTracker {
public:
  // Camera model
  CameraProperty *camera_property = nullptr;

  // Features
  FrameID counter_frame_id = -1;
  FeatureContainer features;
  Features unmatched;
  Features fea_ref;

  // Image, feature, unmatched features book keeping
  cv::Mat img_cur;
  cv::Mat img_ref;

  // Matcher
  cv::Size img_size;
  GMSMatcher matcher;

  // Settings
  bool show_matches = false;

  FeatureTracker();
  FeatureTracker(CameraProperty *camera_property);
  FeatureTracker(CameraProperty *camera_property,
                 const size_t min_track_length,
                 const size_t max_track_length);
  virtual ~FeatureTracker();

  /**
   * Configure
   *
   * @param config_file Path to config file
   */
  virtual int configure(const std::string &config_file) = 0;

  /**
   * Purge old feature tracks
   *
   * @param n N-number of feature tracks to purge (starting with oldest)
   * @returns 0 for success, -1 for failure
   */
  std::vector<FeatureTrack> purge(const size_t n);

  /**
   * Convert list of features to keypoints and descriptors
   *
   * @param features List of features
   * @param keypoints Keypoints
   * @param descriptors Descriptors
   */
  void getKeyPointsAndDescriptors(const Features &features,
                                  std::vector<cv::KeyPoint> &keypoints,
                                  cv::Mat &descriptors);

  /**
   * Convert keypoints and descriptors to list of features
   *
   * @param features List of features
   * @param keypoints Keypoints
   * @param descriptors Descriptors
   */
  void getFeatures(const std::vector<cv::KeyPoint> &keypoints,
                   const cv::Mat &descriptors,
                   Features &features);

  /**
   * Get lost feature tracks
   */
  std::vector<FeatureTrack> getLostTracks();

  /**
   * Detect features
   *
   * @param image Input image
   * @param features List of features
   * @returns 0 for success, -1 for failure
   */
  virtual int detect(const cv::Mat &image, Features &features) = 0;

  /**
   * Match features to feature tracks
   *
   * The idea is that with the current features, we want to match it against
   * the current list of FeatureTrack.
   *
   * @param f1 List of features in current frame
   * @param matches List of matches
   * @returns 0 for success, -1 for failure
   */
  virtual int match(const Features &f1, std::vector<cv::DMatch> &matches);

  /**
   * Initialize feature tracker
   *
   * @param img_cur Current image frame
   * @returns 0 for success, -1 for failure
   */
  virtual int initialize(const cv::Mat &img_cur);

  /**
   * Update feature tracker
   *
   * @param img_cur Current image frame
   * @returns 0 for success, -1 for failure
   */
  virtual int update(const cv::Mat &img_cur);
};

/**
  * FeatureTracker to string
  */
std::ostream &operator<<(std::ostream &os, const FeatureTracker &tracker);

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_TRACKER_HPP
