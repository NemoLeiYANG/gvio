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
#include "gvio/feature2d/klt_tracker.hpp"

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
  KLTTracker tracker0;
  KLTTracker tracker1;

  StereoTracker();
  virtual ~StereoTracker();

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
#endif // GVIO_FEATURE2D_STEREO_TRACKER_HPP
