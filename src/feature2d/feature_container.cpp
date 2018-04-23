#include "gvio/feature2d/feature_container.hpp"

namespace gvio {

FeatureContainer::FeatureContainer() {}

FeatureContainer::FeatureContainer(const size_t min_track_length,
                                   const size_t max_track_length)
    : min_track_length{min_track_length}, max_track_length{max_track_length} {}

int FeatureContainer::addTrack(const FrameID &frame_id,
                               Feature &f0,
                               Feature &f1) {
  // Update and get track and frame ids
  const TrackID track_id = this->counter_track_id;

  // Update features with track ids
  f0.setTrackID(track_id);
  f1.setTrackID(track_id);

  // Add feature track
  auto track = FeatureTrack(track_id, frame_id, f0, f1);
  this->tracking.push_back(track_id);
  this->buffer.emplace(track_id, track);

  this->counter_track_id++;
  return 0;
}

int FeatureContainer::addStereoTrack(const FrameID &frame_id,
                                     Feature &cam0_f0,
                                     Feature &cam0_f1,
                                     Feature &cam1_f0,
                                     Feature &cam1_f1) {
  // Update and get track and frame ids
  const TrackID track_id = this->counter_track_id;

  // Update features with track ids
  cam0_f0.setTrackID(track_id);
  cam0_f1.setTrackID(track_id);
  cam1_f0.setTrackID(track_id);
  cam1_f1.setTrackID(track_id);

  // Add feature track
  auto track = FeatureTrack(track_id,
                            frame_id - 1,
                            frame_id,
                            Features{cam0_f0, cam0_f1},
                            Features{cam1_f0, cam1_f1});
  this->tracking.push_back(track_id);
  this->buffer.emplace(track_id, track);

  this->counter_track_id++;
  return 0;
}

int FeatureContainer::removeTrack(const TrackID &track_id, const bool lost) {
  // Make sure track id is in the buffer
  auto buf_index = this->buffer.find(track_id);
  if (buf_index == this->buffer.end()) {
    return -1;
  }

  // Remove from tracking
  auto t_index =
      std::remove(this->tracking.begin(), this->tracking.end(), track_id);
  if (t_index != this->tracking.end()) {
    this->tracking.erase(t_index);
  }

  // Mark as lost or remove from buffer
  auto track = this->buffer.at(track_id);
  if (track.trackedLength() >= this->min_track_length && lost) {
    this->lost.push_back(track_id);
  } else {
    this->buffer.erase(buf_index);
  }

  return 0;
}

int FeatureContainer::removeTracks(const std::vector<TrackID> &track_ids,
                                   const bool lost) {
  for (auto track_id : track_ids) {
    if (this->removeTrack(track_id, lost) != 0) {
      return -1;
    }
  }

  return 0;
}

void FeatureContainer::removeLostTracks(std::vector<FeatureTrack> &tracks) {
  tracks.clear();
  auto lost_ids = this->lost;
  auto tracking_ids = this->tracking;

  // Remove tracks that are too old (assume lost)
  std::vector<TrackID> tracking_new;
  for (size_t i = 0; i < tracking_ids.size(); i++) {
    TrackID track_id = tracking_ids[i];
    if (this->buffer[track_id].trackedLength() > this->max_track_length) {
      tracks.emplace_back(this->buffer[track_id]);
      this->buffer.erase(this->buffer.find(track_id));
    } else {
      tracking_new.push_back(track_id);
    }
  }
  this->tracking = tracking_new;

  // Remove lost tracks from buffer
  for (size_t i = 0; i < lost_ids.size(); i++) {
    TrackID track_id = lost_ids[i];
    tracks.emplace_back(this->buffer[track_id]);
    this->buffer.erase(this->buffer.find(track_id));
  }

  this->lost.clear();
}

int FeatureContainer::updateTrack(const FrameID frame_id,
                                  const TrackID &track_id,
                                  Feature &f) {
  // Make sure track id is in the buffer
  auto index = this->buffer.find(track_id);
  if (index == this->buffer.end()) {
    return -1;
  }

  // Update track
  auto &track = this->buffer.at(track_id);
  f.setTrackID(track_id);
  track.update(frame_id, f);

  // Mark as lost - too old
  if (track.trackedLength() >= this->max_track_length) {
    this->removeTrack(track.track_id, true);
    return 1;
  }

  return 0;
}

int FeatureContainer::updateStereoTrack(const FrameID frame_id,
                                        const TrackID &track_id,
                                        Feature &cam0_f,
                                        Feature &cam1_f) {
  // Make sure track id is in the buffer
  auto index = this->buffer.find(track_id);
  if (index == this->buffer.end()) {
    return -1;
  }

  // Update track
  auto &track = this->buffer.at(track_id);
  cam0_f.setTrackID(track_id);
  cam1_f.setTrackID(track_id);
  track.updateStereo(frame_id, cam0_f, cam1_f);

  // Mark as lost - too old
  if (track.trackedLength() >= this->max_track_length) {
    this->removeTrack(track.track_id, true);
    return 1;
  }

  return 0;
}

std::vector<Feature> FeatureContainer::getFeaturesTracking() {
  std::vector<Feature> features;

  for (size_t i = 0; i < this->tracking.size(); i++) {
    const TrackID track_id = this->tracking[i];
    const Feature feature = this->buffer[track_id].last();
    features.push_back(feature);
  }

  return features;
}

std::vector<cv::Point2f> FeatureContainer::getKeyPointsTracking() {
  std::vector<cv::Point2f> keypoints;

  for (size_t i = 0; i < this->tracking.size(); i++) {
    const TrackID track_id = this->tracking[i];
    const Feature feature = this->buffer[track_id].last();
    keypoints.push_back(feature.kp.pt);
  }

  return keypoints;
}

std::vector<FeatureTrack> FeatureContainer::purge(const size_t n) {
  std::vector<FeatureTrack> tracks;
  std::vector<TrackID> remove_track_ids;

  /**
   * Note: Map is ordered (source: https://stackoverflow.com/q/7648756/154688)
   */
  size_t counter = 0;
  for (auto kv : this->buffer) {
    auto track_id = kv.first;
    auto track = kv.second;

    remove_track_ids.push_back(track_id);
    tracks.push_back(track);

    counter++;
    if (counter == n) {
      break;
    }
  }

  for (auto track_id : remove_track_ids) {
    this->buffer.erase(track_id);
  }

  return tracks;
}

} // namespace gvio
