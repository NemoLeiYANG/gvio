#include "gvio/feature2d/feature_tracker.hpp"

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

int FeatureTracker::detect(const cv::Mat &image,
                           std::vector<Feature> &features) {
  // Detect features
  std::vector<cv::KeyPoint> keypoints;
  cv::FAST(image,
           keypoints,
           this->fast_threshold,
           this->fast_nonmax_suppression);
  if (keypoints.size() == 0) {
    return -1;
  }

  // Feature descriptor extraction
  cv::Mat mask;
  cv::Mat descriptors;
  this->orb->compute(image, keypoints, descriptors);

  // Create features
  for (int i = 0; i < descriptors.cols; i++) {
    features.emplace_back(keypoints[i], descriptors.col(i));
  }

  return 0;
}

int FeatureTracker::match(const std::vector<Feature> &f1) {
  // Stack previously unmatched features with tracked features
  std::vector<Feature> f0;
  f0.reserve(this->fea_ref.size() + this->unmatched.size());
  f0.insert(f0.end(), this->fea_ref.begin(), this->fea_ref.end());
  f0.insert(f0.end(), this->unmatched.begin(), this->unmatched.end());

  // Convert list of features to list of cv2.KeyPoint and descriptors
  std::vector<cv::KeyPoint> kps0, kps1;
  cv::Mat des0, des1;
  f2kd(f0, kps0, des0);
  f2kd(f1, kps1, des1);

  // Perform matching
  // Note: arguments to the brute-force matcher is (query descriptors,
  // train descriptors), here we use des1 as the query descriptors because
  // des1 represents the latest descriptors from the latest image frame
  std::vector<cv::DMatch> matches;
  this->matcher.match(kps0, des0, kps1, des1, this->img_size, matches);
  std::vector<Feature> features0;
  std::vector<Feature> features1;
  kd2f(kps0, des0, features0);
  kd2f(kps1, des1, features1);

  // Update or add feature track
  std::map<int, bool> tracks_updated;
  for (size_t i = 0; i < matches.size(); i++) {
    const int f0_idx = matches[i].trainIdx;
    const int f1_idx = matches[i].queryIdx;
    auto fea0 = features0[f0_idx];
    auto fea1 = features1[f1_idx];

    if (fea0.track_id != -1) {
      this->updateTrack(fea0.track_id, fea1);
    } else {
      this->addTrack(fea0, fea1);
    }

    tracks_updated.insert({fea0.track_id, true});
  }

  // Drop dead feature tracks
  size_t t_size = this->tracking.size();
  for (size_t i = 0; i < t_size; i++) {
    const auto track_id = this->tracking[i];
    if (tracks_updated.count(track_id) == 0) {
      this->removeTrack(track_id, true);
    }
  }

  return 0;
}

std::vector<FeatureTrack> FeatureTracker::purge(const size_t n) {
  std::vector<FeatureTrack> tracks;

  const int purge_size = std::max(n, this->buffer.size());
  tracks.resize(purge_size);

  /**
   * Note: Map is ordered (source: https://stackoverflow.com/q/7648756/154688)
   */
  int counter = 0;
  for (auto kv : this->buffer) {
    auto track_id = kv.first;
    auto track = kv.second;

    tracks.push_back(track);
    this->buffer.erase(track_id);

    counter++;
    if (counter == purge_size) {
      break;
    }
  }

  return tracks;
}

} // namespace gvio
