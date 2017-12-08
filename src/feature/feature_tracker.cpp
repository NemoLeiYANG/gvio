#include "gvio/feature/feature_tracker.hpp"

namespace gvio {

// int FeatureTracker::configure(const std::string &config_file) { return 0; }

int FeatureTracker::addTrack(Feature &f1, Feature &f2) {
  // Update and get track and frame ids
  this->counter_track_id += 1;
  const TrackID track_id = this->counter_track_id;
  const FrameID frame_id = this->counter_frame_id;

  // Update features with track ids
  f1.setTrackID(track_id);
  f2.setTrackID(track_id);

  // Add feature track
  auto track = FeatureTrack(track_id, frame_id, f1, f2);
  this->tracking.push_back(track_id);
  this->buffer.emplace(track_id, track);

  return 0;
}

int FeatureTracker::removeTrack(const TrackID &track_id, const bool lost) {
  // Make sure track id is in the buffer
  auto index = this->buffer.find(track_id);
  if (index == this->buffer.end()) {
    return -1;
  }

  // Remove from tracking
  this->tracking.erase(
      std::remove(this->tracking.begin(), this->tracking.end(), track_id));

  // Mark as lost or remove from buffer
  if (lost) {
    this->lost.push_back(track_id);
  } else {
    this->buffer.erase(this->buffer.find(track_id));
  }

  return 0;
}

int FeatureTracker::updateTrack(const TrackID &track_id, Feature &f) {
  // Make sure track id is in the buffer
  auto index = this->buffer.find(track_id);
  if (index == this->buffer.end()) {
    return -1;
  }

  // Update track
  f.setTrackID(track_id);
  this->buffer.at(track_id).update(this->counter_frame_id, f);

  return 0;
}

int FeatureTracker::detect(const cv::Mat &image) {
  // Detect features
  std::vector<cv::KeyPoint> keypoints;
  cv::FAST(image,
           keypoints,
           this->fast_threshold,
           this->fast_nonmax_suppression);
  if (keypoints.size() > 0) {
    return -1;
  }

  // Feature descriptor extraction
  cv::Mat mask;
  cv::Mat descriptors;
  this->orb->compute(image, keypoints, descriptors);

  return 0;
}

} // namespace gvio
