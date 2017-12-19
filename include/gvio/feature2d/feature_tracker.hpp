/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_FEATURE_TRACKER_HPP
#define GVIO_FEATURE2D_FEATURE_TRACKER_HPP

#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gvio/util/util.hpp"
#include "gvio/feature2d/gms_matcher.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

// Track and frame ID typedefs
using TrackID = long int;
using FrameID = long int;

/**
 * Feature
 */
struct Feature {
  TrackID track_id = -1;
  cv::KeyPoint kp;
  cv::Mat desc;

  Feature() {}
  Feature(const Vec2 &pt) : kp{cv::Point2f(pt(0), pt(1)), 1.0f} {}
  Feature(const cv::Point2f &pt) : kp{pt, 1.0f} {}
  Feature(const cv::KeyPoint &kp) : kp{kp} {}
  Feature(const cv::KeyPoint &kp, const cv::Mat &desc) : kp{kp}, desc{desc} {}
  Feature(const Feature &f) : track_id{f.track_id}, kp{f.kp}, desc{f.desc} {}

  /**
   * Set feature track ID
   *
   * @param track_id Track ID
   */
  void setTrackID(const TrackID &track_id) { this->track_id = track_id; }

  /**
   * Return feature as cv::KeyPoint
   */
  cv::KeyPoint &getKeyPoint() { return this->kp; }

  /**
   * Feature to string
   */
  friend std::ostream &operator<<(std::ostream &os, const Feature &f) {
    os << "track_id: " << f.track_id << std::endl;
    os << "kp: (" << f.kp.pt.x << ", " << f.kp.pt.y << ")" << std::endl;
    os << "desc: " << f.desc.size() << std::endl;
    return os;
  }
};

/**
 * Compare feature by keypoint
 */
struct CompareFeatureByKeyPoint {
  bool operator()(const Feature &a, const Feature &b) const {
    const double dx = b.kp.pt.x - a.kp.pt.x;
    const double dy = b.kp.pt.y - a.kp.pt.y;
    const double dist = sqrtf(dx * dx + dy * dy);
    return dist;
  }
};

/**
 * Feature track
 */
struct FeatureTrack {
  TrackID track_id = -1;
  FrameID frame_start = -1;
  FrameID frame_end = -1;
  std::vector<Feature> track;

  FeatureTrack() {}

  FeatureTrack(const TrackID &track_id,
               const FrameID &frame_id,
               const Feature &f1,
               const Feature &f2)
      : track_id{track_id}, frame_start{frame_id - 1}, frame_end{frame_id},
        track{f1, f2} {}

  /**
   * Update feature track
   *
   * @param frame_id Frame ID
   * @param data Feature
   */
  void update(const FrameID &frame_id, const Feature &data) {
    this->frame_end = frame_id;
    this->track.push_back(data);
  }

  /**
   * Return last feature seen
   *
   * @returns Last feature
   */
  Feature &last() { return this->track.back(); }

  /**
   * Return feature track length
   *
   * @returns Size of feature track
   */
  size_t tracked_length() { return this->track.size(); }

  /**
   * FeatureTrack to string
   */
  friend std::ostream &operator<<(std::ostream &os, const FeatureTrack &track) {
    os << "track_id: " << track.track_id << std::endl;
    os << "frame_start: " << track.frame_start << std::endl;
    os << "frame_end: " << track.frame_end << std::endl;
    os << "length: " << track.track.size() << std::endl;
    return os;
  }
};

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
  std::vector<Feature> fea_ref;
  std::vector<Feature> unmatched;

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
  int removeTrack(const TrackID &track_id, const bool lost = false);

  /**
   * Update feature track
   *
   * @param track_id Track ID
   * @param f Feature
   * @returns 0 for success, -1 for failure
   */
  int updateTrack(const TrackID &track_id, Feature &f);

  /**
   * Convert list of features to keypoints and descriptors
   *
   * @param features List of features
   * @param keypoints Keypoints
   * @param descriptors Descriptors
   */
  void getKeyPointsAndDescriptors(const std::vector<Feature> &features,
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
                   std::vector<Feature> &features);

  /**
   * Detect features
   *
   * @param image Input image
   * @param features List of features
   * @returns 0 for success, -1 for failure
   */
  virtual int detect(const cv::Mat &image, std::vector<Feature> &features);

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
  cv::Mat drawMatches(const cv::Mat &img0,
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
  cv::Mat drawFeatures(const cv::Mat &image,
                       const std::vector<Feature> features);

  /**
   * Match features to feature tracks
   *
   * The idea is that with the current features, we want to match it against
   * the current list of FeatureTrack.
   *
   * @param f1 List of features in current frame
   * @returns 0 for success, -1 for failure
   */
  virtual int match(const std::vector<Feature> &f1,
                    std::vector<cv::DMatch> &matches);

  /**
   * Purge old feature tracks
   *
   * @param n N-number of feature tracks to purge (starting with oldest)
   * @returns 0 for success, -1 for failure
   */
  std::vector<FeatureTrack> purge(const size_t n);

  /**
   * Initialize feature tracker
   *
   * @param img_cur Current image frame
   * @returns 0 for success, -1 for failure
   */
  int initialize(const cv::Mat &img_cur);

  /**
   * Update feature tracker
   *
   * @param img_cur Current image frame
   * @param show_matches Show matches
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

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_TRACKER_HPP
