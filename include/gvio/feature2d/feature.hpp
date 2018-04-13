/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_FEATURE_HPP
#define GVIO_FEATURE2D_FEATURE_HPP

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Track ID
 */
using TrackID = long int;

/**
 * Feature
 */
struct Feature {
  TrackID track_id = -1;
  cv::KeyPoint kp;
  cv::Mat desc;

  Vec3 ground_truth = Vec3::Zero();

  Feature();
  Feature(const Vec2 &pt);
  Feature(const Vec2 &pt, const Vec3 &ground_truth);
  Feature(const cv::Point2f &pt);
  Feature(const cv::KeyPoint &kp);
  Feature(const cv::KeyPoint &kp, const cv::Mat &desc);

  /**
   * Set feature track ID
   *
   * @param track_id Track ID
   */
  void setTrackID(const TrackID &track_id);

  /**
   * Return feature as Vec2
   */
  Vec2 getKeyPoint();

  /**
   * Return feature as Vec2
   */
  Vec2 getKeyPoint() const;
};

/**
  * Feature to string
  */
std::ostream &operator<<(std::ostream &os, const Feature &f);

/**
 * Features
 */
using Features = std::vector<Feature>;

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_HPP
