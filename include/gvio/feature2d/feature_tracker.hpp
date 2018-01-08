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
#include "gvio/feature2d/feature.hpp"
#include "gvio/feature2d/feature_track.hpp"
#include "gvio/feature2d/gms_matcher.hpp"

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

  // Frame and track ounters
  FrameID counter_frame_id = -1;
  TrackID counter_track_id = -1;

  // Feature track book keeping
  std::vector<TrackID> tracking;
  std::vector<TrackID> lost;
  std::map<TrackID, FeatureTrack> buffer;

  // Image, feature, unmatched features book keeping
  cv::Mat img_cur;
  cv::Mat img_ref;
  Features fea_ref;
  Features unmatched;

  FeatureTracker() {}

  /**
   * Add feature track
   *
   * @param f1 First feature
   * @param f2 Second feature
   * @returns 0 for success, -1 for failure
   */
  int addTrack(Feature &f1, Feature &f2);

  /**
   * Remove feature track
   *
   * @param track_id Track ID
   * @param lost Mark feature track as lost
   * @returns 0 for success, -1 for failure
   */
  int removeTrack(const TrackID &track_id, const bool lost = true);

  /**
   * Update feature track
   *
   * @param track_id Track ID
   * @param f Feature
   * @returns 0 for success, -1 for failure
   */
  int updateTrack(const TrackID &track_id, Feature &f);

  /**
   * Get lost feature tracks
   *
   * @param tracks Lost feature tracks
   */
  void getLostTracks(std::vector<FeatureTrack> &tracks);

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
    for (auto track_id : tracker.tracking) {
      os << track_id << ", ";
    }
    if (tracker.tracking.size()) {
      os << "\b\b]" << std::endl;
    } else {
      os << "]" << std::endl;
    }

    os << "lost : [";
    for (auto track_id : tracker.lost) {
      os << track_id << ", ";
    }
    if (tracker.lost.size()) {
      os << "\b\b]" << std::endl;
    } else {
      os << "]" << std::endl;
    }

    os << "buffer: [";
    for (auto kv : tracker.buffer) {
      os << kv.first << ", ";
    }
    if (tracker.buffer.size()) {
      os << "\b\b]" << std::endl;
    } else {
      os << "]" << std::endl;
    }

    return os;
  }
};

/**
  * Draw tracks
  *
  * @param img_cur Current image frame
  * @param p0 Previous corners
  * @param p1 Current corners
  * @param status Corners status
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_tracks(const cv::Mat &img_cur,
                    const std::vector<cv::Point2f> p0,
                    const std::vector<cv::Point2f> p1,
                    const std::vector<uchar> &status);

/**
  * Draw matches
  *
  * @param img0 Previous image frame
  * @param img1 Current image frame
  * @param k0 Previous keypoints
  * @param k1 Current keypoints
  * @param matches Feature matches
  * @returns Image with feature matches between previous and current frame
  */
cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches);

/**
  * Draw features
  *
  * @param image Image frame
  * @param features List of features
  * @returns Features image
  */
cv::Mat draw_features(const cv::Mat &image, const Features features);

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_TRACKER_HPP
