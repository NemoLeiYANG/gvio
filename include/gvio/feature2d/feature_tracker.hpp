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
#include "gvio/feature2d/draw.hpp"
#include "gvio/feature2d/feature.hpp"
#include "gvio/feature2d/feature_track.hpp"
#include "gvio/feature2d/feature_container.hpp"
#include "gvio/feature2d/gms_matcher.hpp"
#include "gvio/camera/camera_model.hpp"
#include "gvio/camera/pinhole_model.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

class FeatureTracker {
public:
  bool show_matches = false;

  // Matcher
  cv::Size img_size;
  GMSMatcher matcher;

  // Features
  FrameID counter_frame_id = -1;
  FeatureContainer features;
  Features unmatched;
  Features fea_ref;

  // Image, feature, unmatched features book keeping
  cv::Mat img_cur;
  cv::Mat img_ref;

  // Camera model
  const CameraModel *camera_model = nullptr;

  FeatureTracker() {}
  FeatureTracker(const CameraModel *camera_model)
      : camera_model{camera_model} {}
  virtual ~FeatureTracker();

  /**
   * Configure
   *
   * @param config_file Path to config file
   */
  virtual int configure(const std::string &config_file);

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
  virtual int detect(const cv::Mat &image, Features &features);

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

  /**
   * FeatureTracker to string
   */
  friend std::ostream &operator<<(std::ostream &os,
                                  const FeatureTracker &tracker) {
    os << "tracking: [";
    for (auto track_id : tracker.features.tracking) {
      os << track_id << ", ";
    }
    if (tracker.features.tracking.size()) {
      os << "\b\b]" << std::endl;
    } else {
      os << "]" << std::endl;
    }

    os << "lost : [";
    for (auto track_id : tracker.features.lost) {
      os << track_id << ", ";
    }
    if (tracker.features.lost.size()) {
      os << "\b\b]" << std::endl;
    } else {
      os << "]" << std::endl;
    }

    os << "buffer: [";
    for (auto kv : tracker.features.buffer) {
      os << kv.first << ", ";
    }
    if (tracker.features.buffer.size()) {
      os << "\b\b]" << std::endl;
    } else {
      os << "]" << std::endl;
    }

    return os;
  }
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_TRACKER_HPP
