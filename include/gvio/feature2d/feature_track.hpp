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

// Track ID
using FrameID = long int;

// Feature track type
#define MONO_TRACK 0
#define STEREO_TRACK 1

/**
 * Feature track
 */
struct FeatureTrack {
  int type = MONO_TRACK;

  // General
  TrackID track_id = -1;
  FrameID frame_start = -1;
  FrameID frame_end = -1;
  TrackID related = -1;

  // Monocular camera
  Features track;

  // Stereo camera
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
   * Update stereo feature track
   *
   * @param frame_id Frame ID
   * @param cam0_f Feature from camera 0
   * @param cam1_f Feature from camera 1
   */
  void updateStereo(const FrameID &frame_id,
                    const Feature &cam0_f,
                    const Feature &cam1_f);

  /**
   * Slice feature track
   *
   * @param frame_start New frame start
   * @param frame_end New frame end
   */
  void slice(const size_t frame_start, const size_t frame_end);

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
