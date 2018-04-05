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

struct FeatureContainer {
  TrackID counter_track_id = 0;
  size_t max_track_length = 30;

  std::vector<TrackID> tracking;
  std::vector<TrackID> lost;
  std::map<TrackID, FeatureTrack> buffer;

  /**
   * Add feature track
   *
   * @param frame_id Frame ID
   * @param f1 First feature
   * @param f2 Second feature
   * @returns 0 for success, -1 for failure
   */
  int addTrack(const FrameID &frame_id, Feature &f1, Feature &f2);

  /**
   * Remove feature track
   *
   * @param track_id Track ID
   * @param lost Mark feature track as lost
   * @returns 0 for success, -1 for failure
   */
  int removeTrack(const TrackID &track_id, const bool lost = true);

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
