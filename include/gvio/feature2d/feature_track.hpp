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
  TrackID related = -1;

  // Monocular camera setup
  Features track;

  // Stereo camera setup
  Features track0;
  Features track1;

  FeatureTrack();
  FeatureTrack(const TrackID &track_id,
               const FrameID &frame_id,
               const Feature &f1,
               const Feature &f2);
  FeatureTrack(const TrackID &track_id,
               const FrameID &frame_start,
               const FrameID &frame_end,
               const Features &tracks0,
               const Features &tracks1);

  /**
   * Update feature track
   *
   * @param frame_id Frame ID
   * @param data Feature
   */
  void update(const FrameID &frame_id, const Feature &data);

  /**
   * Return last feature seen
   *
   * @returns Last feature
   */
  Feature &last();

  /**
   * Return feature track length
   *
   * @returns Size of feature track
   */
  size_t trackedLength();

  /**
   * Return feature track length
   *
   * @returns Size of feature track
   */
  size_t trackedLength() const;
};

/**
  * FeatureTrack to string
  */
std::ostream &operator<<(std::ostream &os, const FeatureTrack &track);

/**
 * Feature tracks
 */
using FeatureTracks = std::vector<FeatureTrack>;

/**
 * Feature track to CSV file
 *
 * @param track Feature track
 * @param output_path Output path
 */
int save_feature_track(const FeatureTrack &track,
                       const std::string &output_path);

/**
 * Feature tracks to CSV file
 *
 * @param tracks Feature track
 * @param output_path Output path
 */
int save_feature_tracks(const FeatureTracks &tracks,
                        const std::string &output_path);

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_TRACK_HPP
