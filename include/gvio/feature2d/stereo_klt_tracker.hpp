/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_STEREO_KLT_TRACKER_HPP
#define GVIO_FEATURE2D_STEREO_KLT_TRACKER_HPP

#include "gvio/camera/pinhole_model.hpp"
#include "gvio/feature2d/feature_tracker.hpp"
#include "gvio/feature2d/feature_container.hpp"
#include "gvio/feature2d/klt_tracker.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Stereo feature tracker
 */
class StereoKLTTracker {
public:
  CameraProperty camprop0;
  CameraProperty camprop1;
  Mat4 T_cam1_cam0 = I(4);

  TrackID counter_frame_id = -1;
  FeatureContainer features;
  std::vector<cv::Point2f> cam0_pts;
  std::vector<cv::Point2f> cam1_pts;
  std::vector<int> track_ids;
  cv::Mat prev_cam0_img;
  cv::Mat prev_cam1_img;

  // Detector settings
  int max_corners = 1000;
  double quality_level = 0.01;
  double min_distance = 5.0;

  // LK settings
  int pyramid_levels = 3;
  int max_iteration = 30;
  double track_precision = 0.01;

  // Misc settings
  int image_width = 0;
  int image_height = 0;
  bool show_matches = false;

  // Stats
  struct FeatureContainerStats stats;

  StereoKLTTracker();
  StereoKLTTracker(const CameraProperty &camprop0,
                   const CameraProperty &camprop1,
                   const Mat4 &T_cam1_cam0,
                   const size_t min_track_length,
                   const size_t max_track_length);
  virtual ~StereoKLTTracker();

  /**
   * Detect corners in image
   *
   * @param image Input image
   * @returns Corners detected [px]
   */
  std::vector<cv::Point2f> detect(const cv::Mat &image);

  /**
   * Match features spacially
   *
   * @param img0 First image
   * @param img1 Second image
   * @param pts0 Input points [px]
   * @param pts1 Output points [px]
   * @param mask Mask
   */
  void match(const cv::Mat &img0,
             const cv::Mat &img1,
             std::vector<cv::Point2f> &pts0,
             std::vector<cv::Point2f> &pts1,
             std::vector<uchar> &mask);

  /**
   * Update a feature track
   *
   * @param index Index of feature in StereoKLTTracker.track_ids
   * @param is_inlier Is feature an inlier
   * @param pt0_ref Feature point in camera 0 in previous frame
   * @param pt0_cur Feature point in camera 0 in current frame
   * @param pt1_ref Feature point in camera 1 in previous frame
   * @param pt1_cur Feature point in camera 1 in current frame
   *
   * @returns Track ID if updated or added to container, else returns -1 for
   * removed
   */
  int updateTrack(const int index,
                  const bool is_inlier,
                  const cv::Point2f &pt0_ref,
                  const cv::Point2f &pt0_cur,
                  const cv::Point2f &pt1_ref,
                  const cv::Point2f &pt1_cur);

  /**
   * Track features
   *
   * The idea is that with the current features, we want to match it against
   * the current list of FeatureTrack.
   *
   * @param cam0_img Camera 0 input image
   * @param cam1_img Camera 1 input image
   * @returns 0 for success, -1 for failure
   */
  void trackFeatures(const cv::Mat &img_ref, const cv::Mat &img_cur);

  /**
   * Initialize stereo feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int initialize(const cv::Mat &img0_cur, const cv::Mat &img1_cur);

  /**
   * Replenish features
   *
   * @param image Image
   * @returns 0 for success, -1 for failure
   */
  void replenishFeatures(const cv::Mat &image);

  /**
   * Update feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int update(const cv::Mat &img0_cur, const cv::Mat &img1_cur);

  int update2(const cv::Mat &img0_cur, const cv::Mat &img1_cur, long ts);

  /**
   * Get lost feature tracks
   *
   * @returns List of feature track
   */
  std::vector<FeatureTrack> getLostTracks();
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_STEREO_KLT_TRACKER_HPP
