/**
 * @file
 * @ingroup feature2d
 */
#ifndef GVIO_FEATURE2D_FEATURE_CONTAINER_HPP
#define GVIO_FEATURE2D_FEATURE_CONTAINER_HPP

#include "gvio/feature2d/feature.hpp"
#include "gvio/feature2d/feature_track.hpp"

namespace gvio {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Feature container stats
 */
struct FeatureContainerStats {
  std::vector<int> tracking;
  std::vector<int> lost;

  /**
   * Keep track of features tracking and lost
   *
   * @param tracking Features currently tracking
   * @param lost Features lost
   */
  void update(const int tracking, const int lost);

  /**
   * Output stats to file
   *
   * @param output_path Output path for stats
   */
  int save(const std::string &output_path);
};

/**
 * Feature container
 */
struct FeatureContainer {
  TrackID counter_track_id = 0;
  size_t min_track_length = 10;
  size_t max_track_length = 20;
  std::vector<TrackID> tracking;
  std::vector<TrackID> lost;
  std::map<TrackID, FeatureTrack> buffer;

  FeatureContainer();
  FeatureContainer(const size_t min_track_length,
                   const size_t max_track_length);

  /**
   * Add feature track
   *
   * @param frame_id Frame ID
   * @param f0 First feature
   * @param f1 Second feature
   * @returns 0 for success, -1 for failure
   */
  int addTrack(const FrameID &frame_id, Feature &f0, Feature &f1);

  /**
   * Add stereo feature track
   *
   * @param frame_id Frame ID
   * @param cam0_f0 First feature in camera 0
   * @param cam0_f1 Second feature in camera 0
   * @param cam1_f0 First feature in camera 1
   * @param cam1_f1 Second feature in camera 1
   * @returns 0 for success, -1 for failure
   */
  int addStereoTrack(const FrameID &frame_id,
                     Feature &cam0_f0,
                     Feature &cam0_f1,
                     Feature &cam1_f0,
                     Feature &cam1_f1);

  /**
   * Remove feature track
   *
   * @param track_id Track ID
   * @param lost Mark feature track as lost
   * @returns 0 for success, -1 for failure
   */
  int removeTrack(const TrackID &track_id, const bool lost = true);

  /**
   * Remove feature tracks
   *
   * @param track_ids List of Track ID
   * @param lost Mark feature track as lost
   * @returns 0 for success, -1 for failure
   */
  int removeTracks(const std::vector<TrackID> &track_ids,
                   const bool lost = true);

  /**
   * Remove lost feature tracks
   *
   * @param tracks Lost feature tracks
   */
  void removeLostTracks(std::vector<FeatureTrack> &tracks);

  /**
   * Update feature track
   *
   * @param frame_id Frame ID
   * @param track_id Track ID
   * @param f Feature
   * @returns 0 for success, -1 for failure
   */
  int updateTrack(const FrameID frame_id, const TrackID &track_id, Feature &f);

  /**
   * Update stereo feature track
   *
   * @param frame_id Frame ID
   * @param track_id Track ID
   * @param cam0_f Feature in camera 0
   * @param cam1_f Feature in camera 1
   * @returns 0 for success, -1 for failure
   */
  int updateStereoTrack(const FrameID frame_id,
                        const TrackID &track_id,
                        Feature &cam0_f,
                        Feature &cam1_f);

  /**
   * Get list of features currently tracking
   */
  std::vector<Feature> getFeaturesTracking();

  /**
   * Get list of Keypoints currently tracking
   */
  std::vector<cv::Point2f> getKeyPointsTracking();

  /**
   * Purge old feature tracks
   *
   * @param n N-number of feature tracks to purge (starting with oldest)
   * @returns 0 for success, -1 for failure
   */
  std::vector<FeatureTrack> purge(const size_t n);
};

/** @} group feature2d */
} // namespace gvio
#endif // GVIO_FEATURE2D_FEATURE_CONTAINER_HPP
