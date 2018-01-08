#include "gvio/feature2d/klt_tracker.hpp"

namespace gvio {

int KLTTracker::addTrack(Feature &f1, Feature &f2) {
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

int KLTTracker::removeTrack(const TrackID &track_id, const bool lost) {
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

void KLTTracker::getLostTracks(std::vector<FeatureTrack> &tracks) {
  tracks.clear();
  auto lost_ids = this->lost;

  for (size_t i = 0; i < lost_ids.size(); i++) {
    TrackID track_id = lost_ids[i];
    tracks.emplace_back(this->buffer[track_id]);
    this->buffer.erase(this->buffer.find(track_id));
  }

  this->lost.clear();
}

int KLTTracker::updateTrack(const TrackID &track_id, Feature &f) {
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

std::vector<FeatureTrack> KLTTracker::purge(const size_t n) {
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

int KLTTracker::initialize(const cv::Mat &img_cur) {
  DEBUG("Initialize feature tracker!");
  img_cur.copyTo(this->img_ref);
  this->counter_frame_id++;
  return this->detect(img_cur, this->fea_ref);
}

int KLTTracker::detect(const cv::Mat &image, Features &features) {
  // Convert image to gray scale
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, CV_BGR2GRAY);

  // Feature detection
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(gray_image, corners, 1000, 0.01, 10);

  // Create features
  features.clear();
  for (auto corner : corners) {
    features.emplace_back(corner);
  }

  return 0;
}

int KLTTracker::track(const Features &features) {
  // Convert list of features to list of cv::Point2f
  std::vector<cv::Point2f> p0;
  for (auto f : features) {
    p0.push_back(f.kp.pt);
  }

  // Convert input images to gray scale
  cv::Mat gray_img_ref, gray_img_cur;
  cv::cvtColor(this->img_ref, gray_img_ref, CV_BGR2GRAY);
  cv::cvtColor(this->img_cur, gray_img_cur, CV_BGR2GRAY);

  // Track features
  std::vector<cv::Point2f> p1;
  std::vector<uchar> status;
  std::vector<float> err;
  cv::Size win_size(21, 21);
  cv::calcOpticalFlowPyrLK(gray_img_ref, // Reference image
                           gray_img_cur, // Current image
                           p0,           // Input points
                           p1,           // Output points
                           status,       // Tracking status
                           err,          // Tracking error
                           win_size);    // Window size

  // Show matches
  if (this->show_matches) {
    cv::Mat matches_img = draw_tracks(this->img_cur, p0, p1, status);
    cv::imshow("Matches", matches_img);
  }

  // Update or remove feature track
  Features keeping;
  int index = 0;

  for (auto s : status) {
    auto f0 = features[index];

    if (s == 1) {
      auto f1 = Feature(p1[index]);
      if (f0.track_id == -1) {
        this->addTrack(f0, f1);
      } else {
        this->updateTrack(f0.track_id, f1);
      }
      keeping.push_back(f1);

    } else {
      this->removeTrack(f0.track_id, true);
    }

    index++;
  }
  this->fea_ref = keeping;

  return 0;
}

int KLTTracker::update(const cv::Mat &img_cur) {
  // Keep track of current image
  img_cur.copyTo(this->img_cur);

  // Initialize feature tracker
  if (this->fea_ref.size() == 0) {
    this->initialize(img_cur);
    return 0;
  }

  // Detect
  if (this->fea_ref.size() < 200) {
    if (this->detect(img_cur, this->fea_ref) != 0) {
      return -1;
    }
  }
  this->counter_frame_id++;

  // Match features
  if (this->track(this->fea_ref) != 0) {
    return -2;
  }

  // Update
  img_cur.copyTo(this->img_ref);

  return 0;
}

} // namespace gvio
