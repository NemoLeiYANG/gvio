/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_FEATURE_TRACK_HPP
#define GVIO_FEATURE2D_FEATURE_TRACK_HPP

#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gvio/util/util.hpp"
#include "gvio/feature2d/feature.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

// Track and frame ID typedefs
using FrameID = long int;

/**
 * Feature track
 */
struct FeatureTrack {
  TrackID track_id = -1;
  FrameID frame_start = -1;
  FrameID frame_end = -1;
  Features track;

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
  size_t trackedLength() { return this->track.size(); }

  /**
   * Return feature track length
   *
   * @returns Size of feature track
   */
  size_t trackedLength() const { return this->track.size(); }

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

/**
 * Feature tracks
 */
using FeatureTracks = std::vector<FeatureTrack>;

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_TRACK_HPP
