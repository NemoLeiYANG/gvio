/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_STEREO_TRACKER_HPP
#define GVIO_FEATURE2D_STEREO_TRACKER_HPP

#include "gvio/camera/camera_model.hpp"
#include "gvio/camera/pinhole_model.hpp"
#include "gvio/feature2d/feature_tracker.hpp"
#include "gvio/feature2d/feature_container.hpp"
#include "gvio/feature2d/orb_tracker.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Stereo feature tracker
 */
class StereoTracker {
public:
  ORBTracker tracker0;
  ORBTracker tracker1;
  GMSMatcher matcher;
  TrackID counter_track_id = 0;

  size_t min_track_length = 10;
  size_t max_track_length = 10;
  bool show_matches = false;

  StereoTracker();
  StereoTracker(const size_t min_track_length, const size_t max_track_length);
  virtual ~StereoTracker();

  /**
   * Initialize stereo feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int initialize(const cv::Mat &img0_cur, const cv::Mat &img1_cur);

  /**
   * Update feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int update(const cv::Mat &img0_cur, const cv::Mat &img1_cur);

  /**
   * Get lost feature tracks
   *
   * @returns List of feature track
   */
  std::vector<FeatureTrack> getLostTracks();
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_STEREO_TRACKER_HPP
