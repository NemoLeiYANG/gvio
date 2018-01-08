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
class KLTTracker {
public:
  bool show_matches = false;

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

  KLTTracker() {}

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
   * Initialize feature tracker
   *
   * @param img_cur Current image frame
   * @returns 0 for success, -1 for failure
   */
  int initialize(const cv::Mat &img_cur);

  /**
   * Detect features
   *
   * @param image Input image
   * @param features List of features
   * @returns 0 for success, -1 for failure
   */
  int detect(const cv::Mat &image, Features &features);

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
  int update(const cv::Mat &img_cur);
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_KLT_TRACKER_HPP
