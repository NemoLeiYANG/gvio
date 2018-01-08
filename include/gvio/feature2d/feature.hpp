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
 * Features
 */
using Features = std::vector<Feature>;

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_HPP
