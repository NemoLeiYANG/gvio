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
  Feature::toKeyPointsAndDescriptors(f0, kps0, des0);
  Feature::toKeyPointsAndDescriptors(f1, kps1, des1);

  // Perform matching
  // Note: arguments to the brute-force matcher is (query descriptors,
  // train descriptors), here we use des1 as the query descriptors because
  // des1 represents the latest descriptors from the latest image frame
  std::vector<cv::DMatch> matches_bf;
  // this->matcher.match(kps0, des0, kps1, des1, this->img_size, matches_bf);

  return 0;
}

} // namespace gvio
