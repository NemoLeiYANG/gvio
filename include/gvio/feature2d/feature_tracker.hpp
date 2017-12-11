/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_TRACKER_HPP
#define GVIO_FEATURE2D_TRACKER_HPP

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
  Feature(const cv::KeyPoint &kp) : kp{kp} {}
  Feature(const cv::KeyPoint &kp, const cv::Mat &desc) : kp{kp}, desc{desc} {}

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
      : track_id{track_id}, frame_start{frame_id - 1}, frame_end{frame_id} {
    this->track.push_back(f1);
    this->track.push_back(f2);
  }

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
};

class FeatureTracker {
public:
  bool configured = false;

  // Detector
  int fast_threshold = 10;
  bool fast_nonmax_suppression = true;

  // Descriptor
  cv::Ptr<cv::ORB> orb = cv::ORB::create();

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
  cv::Mat img_ref;
  std::vector<Feature> fea_ref;
  std::vector<Feature> unmatched;

  FeatureTracker() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

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
   * Detect features
   *
   * @param image Input image
   * @param features List of features
   * @returns 0 for success, -1 for failure
   */
  int detect(const cv::Mat &image, std::vector<Feature> &features);

  /**
   * Match features to feature tracks
   *
   * The idea is that with the current features, we want to match it against
   * the current list of FeatureTrack.
   *
   * @param f1 List of features in current frame
   * @returns 0 for success, -1 for failure
   */
  int match(const std::vector<Feature> &f1);

  /**
   * Purge old feature tracks
   *
   * @returns 0 for success, -1 for failure
   */
  std::vector<FeatureTrack> purge(const size_t n);

  /**
   * Update feature tracker
   *
   * @returns 0 for success, -1 for failure
   */
  int update();
};

/**
  * Features to keypoints and descriptors
  */
inline void f2kd(const std::vector<Feature> &features,
                 std::vector<cv::KeyPoint> &keypoints,
                 cv::Mat &descriptors) {
  descriptors = cv::Mat(features[0].desc.rows, features.size(), CV_8UC1);
  for (size_t i = 0; i < features.size(); i++) {
    keypoints.push_back(features[i].kp);

    cv::Mat d;
    features[i].desc.copyTo(d);
    descriptors.col(i) = d;
  }
}

/**
  * Keypoints and descriptors to Features
  */
inline void kd2f(const std::vector<cv::KeyPoint> &keypoints,
                 const cv::Mat &descriptors,
                 std::vector<Feature> &features) {
  for (size_t i = 0; i < keypoints.size(); i++) {
    features.emplace_back(keypoints[i], descriptors.col(i));
  }
}

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_TRACKER_HPP
