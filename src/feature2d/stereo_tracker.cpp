#include "gvio/feature2d/stereo_tracker.hpp"

namespace gvio {

StereoTracker::StereoTracker() {}

StereoTracker::StereoTracker(CameraProperty *camera_property,
                             const size_t min_track_length,
                             const size_t max_track_length)
    : tracker0{camera_property, min_track_length, max_track_length},
      tracker1{camera_property, min_track_length, max_track_length},
      min_track_length{min_track_length}, max_track_length{max_track_length} {}

StereoTracker::~StereoTracker() {}

int StereoTracker::initialize(const cv::Mat &img0_cur,
                              const cv::Mat &img1_cur) {
  int retval = 0;
  retval += this->tracker0.initialize(img0_cur);
  retval += this->tracker1.initialize(img1_cur);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int StereoTracker::update(const cv::Mat &img0_cur, const cv::Mat &img1_cur) {
  // Detect features
  int retval = 0;
  retval += this->tracker0.update(img0_cur);
  retval += this->tracker1.update(img1_cur);
  if (retval != 0) {
    return -1;
  }

  // Obtain features currently tracking
  const std::vector<Feature> f0 = this->tracker0.fea_ref;
  const std::vector<Feature> f1 = this->tracker1.fea_ref;

  // Convert features to keypoints and descriptors (cv::KeyPoint and cv::Mat)
  std::vector<cv::KeyPoint> k0, k1;
  cv::Mat d0, d1;
  this->tracker0.getKeyPointsAndDescriptors(f0, k0, d0);
  this->tracker1.getKeyPointsAndDescriptors(f1, k1, d1);

  // Use matcher to match features
  std::vector<cv::DMatch> matches;
  this->matcher.match(k0, d0, k1, d1, img0_cur.size(), matches);

  // Initialize list of outliers with inliers
  std::vector<TrackID> outliers0 = this->tracker0.features.tracking;
  std::vector<TrackID> outliers1 = this->tracker1.features.tracking;

  // Obtain outliers
  for (size_t i = 0; i < matches.size(); i++) {
    // Get track ids
    const int k0_idx = matches[i].queryIdx;
    const int k1_idx = matches[i].trainIdx;
    const TrackID track0 = f0[k0_idx].track_id;
    const TrackID track1 = f1[k1_idx].track_id;

    // Make sure tracks are not already related
    if (this->tracker0.features.buffer[track0].related != -1) {
      continue;
    }
    if (this->tracker1.features.buffer[track1].related != -1) {
      continue;
    }

    // Mark related, this is to link the feature tracks together later
    this->tracker0.features.buffer[track0].related = track1;
    this->tracker1.features.buffer[track1].related = track0;

    // Remove track ids from outliers
    auto idx0 = std::remove(outliers0.begin(), outliers0.end(), track0);
    auto idx1 = std::remove(outliers1.begin(), outliers1.end(), track1);
    if (idx0 != outliers0.end()) {
      outliers0.erase(idx0);
    }
    if (idx1 != outliers1.end()) {
      outliers1.erase(idx1);
    }
  }

  // Remove outliers
  this->tracker0.features.removeTracks(outliers0);
  this->tracker1.features.removeTracks(outliers1);

  // Visualize matches
  if (this->show_matches) {
    cv::Mat match_img = draw_matches(img0_cur, img1_cur, k0, k1, matches);
    cv::imshow("Matches", match_img);
  }

  return 0;
}

std::vector<FeatureTrack> StereoTracker::getLostTracks() {
  FeatureTracks stereo_tracks;

  for (auto track0_id : this->tracker0.features.lost) {
    // Get track0 and track 1
    auto track0 = this->tracker0.features.buffer[track0_id];
    if (track0.related == -1) {
      continue;
    }
    auto track1 = this->tracker1.features.buffer[track0.related];

    // Slice the matching feature tracks from tracker0 and tracker1 so that
    // they have the same:
    // - frame start
    // - frame end
    // - feature track length
    FrameID frame_start = std::max(track0.frame_start, track1.frame_start);
    FrameID frame_end = std::min(track0.frame_end, track1.frame_end);
    if ((frame_end - frame_start + 1) < (long) this->min_track_length) {
      continue;
    }
    track0.slice(frame_start, frame_end);
    track1.slice(frame_start, frame_end);

    // Assert
    assert(track0.frame_start == track1.frame_start);
    assert(track0.frame_end == track1.frame_end);
    assert(track0.related == track1.track_id);
    assert(track1.related == track0.track_id);
    assert(track0.track.size() == track1.track.size());

    // Combine two feature tracks into a single stereo feature track and add it
    // to list of feature tracks and increment track counter
    stereo_tracks.emplace_back(this->counter_track_id,
                               track0.frame_start,
                               track0.frame_end,
                               track0.track,
                               track1.track);
    this->counter_track_id++;

    // Remove them from tracker0 and tracker1
    this->tracker0.features.removeTrack(track0.track_id, false);
    this->tracker1.features.removeTrack(track1.track_id, false);

    // Assert
    assert(this->tracker0.features.buffer.count(track0.track_id) == 0);
    assert(this->tracker1.features.buffer.count(track1.track_id) == 0);
  }

  return stereo_tracks;
}

} // namespace gvio
