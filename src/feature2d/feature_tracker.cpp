#include "gvio/feature2d/feature_tracker.hpp"

namespace gvio {

int FeatureTracker::addTrack(Feature &f1, Feature &f2) {
  // Update and get track and frame ids
  this->counter_track_id++;
  const TrackID track_id = this->counter_track_id;
  const FrameID frame_id = this->counter_frame_id;
  DEBUG("Add track [%ld]", track_id);

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
  DEBUG("Remove track [%ld]", track_id);
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
  DEBUG("Update track [%ld]", track_id);

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

void FeatureTracker::getKeyPointsAndDescriptors(
    const std::vector<Feature> &features,
    std::vector<cv::KeyPoint> &keypoints,
    cv::Mat &descriptors) {
  descriptors = cv::Mat(features.size(), features[0].desc.cols, CV_8UC1);
  for (size_t i = 0; i < features.size(); i++) {
    keypoints.push_back(features[i].kp);
    features[i].desc.copyTo(descriptors.row(i));
  }
}

void FeatureTracker::getFeatures(const std::vector<cv::KeyPoint> &keypoints,
                                 const cv::Mat &descriptors,
                                 std::vector<Feature> &features) {
  for (size_t i = 0; i < keypoints.size(); i++) {
    features.emplace_back(keypoints[i], descriptors.row(i));
  }
}

int FeatureTracker::detect(const cv::Mat &image,
                           std::vector<Feature> &features) {
  UNUSED(image);
  UNUSED(features);
  LOG_ERROR("You're suppose to override FeatureTracker::detect()!");
  LOG_ERROR("FeatureTracker is a base class!");
  exit(-1);

  return 0;
}

cv::Mat FeatureTracker::drawMatches(const cv::Mat &img0,
                                    const cv::Mat &img1,
                                    const std::vector<cv::KeyPoint> k0,
                                    const std::vector<cv::KeyPoint> k1,
                                    const std::vector<cv::DMatch> &matches) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < matches.size(); i++) {
    const int k0_idx = matches[i].queryIdx;
    const int k1_idx = matches[i].trainIdx;
    cv::KeyPoint p0 = k0[k0_idx];
    cv::KeyPoint p1 = k1[k1_idx];

    // Point 1
    p1.pt.y += img0.rows;

    // Draw circle and line
    cv::circle(match_img, p0.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(match_img, p1.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::line(match_img, p0.pt, p1.pt, cv::Scalar(0, 255, 0));
  }

  return match_img;
}

cv::Mat FeatureTracker::drawFeatures(const cv::Mat &image,
                                     const std::vector<Feature> features) {
  cv::Mat fea_img;

  image.copyTo(fea_img);
  for (auto f : features) {
    cv::circle(fea_img, f.kp.pt, 2, cv::Scalar(0, 255, 0), -1);
  }

  return fea_img;
}

int FeatureTracker::match(const std::vector<Feature> &f1,
                          std::vector<cv::DMatch> &matches) {
  // Stack previously unmatched features with tracked features
  std::vector<Feature> f0;
  f0.reserve(this->fea_ref.size() + this->unmatched.size());
  f0.insert(f0.end(), this->fea_ref.begin(), this->fea_ref.end());
  f0.insert(f0.end(), this->unmatched.begin(), this->unmatched.end());

  // Match features
  // -- Convert list of features to list of cv2.KeyPoint and descriptors
  std::vector<cv::KeyPoint> k0, k1;
  cv::Mat d0, d1;
  this->getKeyPointsAndDescriptors(f0, k0, d0);
  this->getKeyPointsAndDescriptors(f1, k1, d1);

  // -- Perform matching
  // Note: arguments to the brute-force matcher is (query descriptors,
  // train descriptors), here we use d1 as the query descriptors because
  // d1 represents the latest descriptors from the latest image frame
  this->matcher.match(k0, d0, k1, d1, this->img_size, matches);
  DEBUG("Number of matches: %d", (int) matches.size());

  // Show matches
  if (this->show_matches) {
    const cv::Mat matches_img =
        this->drawMatches(this->img_ref, this->img_cur, k0, k1, matches);
    cv::imshow("Matches", matches_img);
  }

  // Update or add feature track
  std::map<int, bool> tracks_updated;
  std::map<int, bool> idx_updated;
  this->fea_ref.clear();

  for (size_t i = 0; i < matches.size(); i++) {
    const int f0_idx = matches[i].queryIdx;
    const int f1_idx = matches[i].trainIdx;
    auto fea0 = f0[f0_idx];
    auto fea1 = f1[f1_idx];

    if (fea0.track_id != -1) {
      this->updateTrack(fea0.track_id, fea1);
    } else {
      this->addTrack(fea0, fea1);
    }

    this->fea_ref.push_back(fea1);
    tracks_updated.insert({fea1.track_id, true});
    idx_updated.insert({f1_idx, true});
  }

  // Drop dead feature tracks
  const std::vector<TrackID> tracks_tracking = this->tracking;
  for (auto track_id : tracks_tracking) {
    if (tracks_updated.count(track_id) == 0) {
      this->removeTrack(track_id, true);
    }
  }

  // Update list of reference and unmatched features
  this->unmatched.clear();
  for (size_t i = 0; i < f1.size(); i++) {
    if (idx_updated.find(i) == idx_updated.end()) {
      this->unmatched.push_back(f1[i]);
    }
  }

  return 0;
}

std::vector<FeatureTrack> FeatureTracker::purge(const size_t n) {
  std::vector<FeatureTrack> tracks;

  /**
   * Note: Map is ordered (source: https://stackoverflow.com/q/7648756/154688)
   */
  size_t counter = 0;
  for (auto kv : this->buffer) {
    auto track_id = kv.first;
    auto track = kv.second;

    tracks.push_back(track);
    this->buffer.erase(track_id);

    counter++;
    if (counter == n) {
      break;
    }
  }

  return tracks;
}

int FeatureTracker::initialize(const cv::Mat &img_cur) {
  DEBUG("Initialize feature tracker!");
  img_cur.copyTo(this->img_ref);
  this->img_size = img_cur.size();
  return this->detect(img_cur, this->fea_ref);
}

int FeatureTracker::update(const cv::Mat &img_cur) {
  // Keep track of current image
  img_cur.copyTo(this->img_cur);

  // Initialize feature tracker
  if (this->fea_ref.size() == 0) {
    this->initialize(img_cur);
    return 0;
  }

  // Detect
  std::vector<Feature> features;
  if (this->detect(img_cur, features) != 0) {
    return -1;
  }

  // Match features
  std::vector<cv::DMatch> matches;
  if (this->match(features, matches) != 0) {
    return -2;
  }

  // Update
  img_cur.copyTo(this->img_ref);

  return 0;
}

} // namespace gvio
