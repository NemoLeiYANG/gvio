#include "gvio/feature2d/stereo_orb_tracker.hpp"

namespace gvio {

StereoORBTracker::StereoORBTracker() {}

StereoORBTracker::StereoORBTracker(const CameraProperty &camprop0,
                                   const CameraProperty &camprop1,
                                   const size_t min_track_length,
                                   const size_t max_track_length)
    : tracker0{camprop0, min_track_length, max_track_length},
      tracker1{camprop1, min_track_length, max_track_length},
      min_track_length{min_track_length}, max_track_length{max_track_length} {}

StereoORBTracker::~StereoORBTracker() {}

int StereoORBTracker::initialize(const cv::Mat &cam0_img,
                                 const cv::Mat &cam1_img) {
  int retval = 0;
  retval += this->tracker0.initialize(cam0_img);
  retval += this->tracker1.initialize(cam1_img);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int StereoORBTracker::update(const cv::Mat &cam0_img, const cv::Mat &cam1_img) {
  // Detect features
  int retval = 0;
  retval += this->tracker0.update(cam0_img);
  retval += this->tracker1.update(cam1_img);
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
  auto matcher = GMSMatcher();
  matcher.match(k0, d0, k1, d1, cam0_img.size(), matches);

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
    cv::Mat match_img = draw_matches(cam0_img, cam1_img, k0, k1, matches);
    cv::imshow("Matches", match_img);
    cv::waitKey(1);
  }

  return 0;
}

int StereoORBTracker::update2(const cv::Mat &cam0_img,
                              const cv::Mat &cam1_img,
                              long ts) {
  UNUSED(ts);
  return this->update(cam0_img, cam1_img);
}

std::vector<FeatureTrack> StereoORBTracker::getLostTracks() {
  FeatureTracks stereo_tracks;

  auto tracker0_lost_tracks = this->tracker0.features.lost;
  for (auto track0_id : tracker0_lost_tracks) {
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

    // Remove them from tracker0 and tracker1
    this->tracker0.features.removeTrack(track0.track_id, false);
    this->tracker1.features.removeTrack(track1.track_id, false);
    assert(this->tracker0.features.buffer.count(track0.track_id) == 0);
    assert(this->tracker1.features.buffer.count(track1.track_id) == 0);
    assert(std::count(this->tracker0.features.lost.begin(),
                      this->tracker0.features.lost.end(),
                      track0.track_id) == 0);
    assert(std::count(this->tracker1.features.lost.begin(),
                      this->tracker1.features.lost.end(),
                      track1.track_id) == 0);

    std::cout << "--> lost: " << this->tracker0.features.lost.size()
              << std::endl;

    this->counter_track_id++;
  }

  // Clear lost tracks
  // this->tracker0.features.lost.clear();
  // this->tracker1.features.lost.clear();
  assert(this->tracker0.features.lost.size() == 0);
  assert(this->tracker1.features.lost.size() == 0);

  return stereo_tracks;
}

} // namespace gvio
