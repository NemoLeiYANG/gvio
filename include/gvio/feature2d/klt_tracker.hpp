/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_KLT_TRACKER_HPP
#define GVIO_FEATURE2D_KLT_TRACKER_HPP

#include "gvio/feature2d/feature_tracker.hpp"
#include "gvio/feature2d/feature_container.hpp"
#include "gvio/camera/camera_model.hpp"
#include "gvio/camera/pinhole_model.hpp"

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
  FrameID counter_frame_id = -1;
  cv::Mat img_cur;
  cv::Mat img_ref;
  FeatureContainer features;

  int nb_max_corners = 500;
  double quality_level = 0.01;
  double min_distance = 10.0;
  bool show_matches = false;

  const CameraModel *camera_model = nullptr;

  KLTTracker() {}

  KLTTracker(const CameraModel *camera_model) : camera_model{camera_model} {}

  KLTTracker(const int nb_max_corners,
             const double quality_level,
             const double min_distance,
             const CameraModel *camera_model)
      : nb_max_corners{nb_max_corners}, quality_level{quality_level},
        min_distance{min_distance}, camera_model{camera_model} {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Get lost feature tracks
   *
   * @param tracks Lost feature tracks
   */
  std::vector<FeatureTrack> getLostTracks();

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
