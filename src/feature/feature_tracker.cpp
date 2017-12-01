#include "gvio/feature/feature_tracker.hpp"

namespace gvio {

// int FeatureTracker::configure(const std::string &config_file) { return 0; }

int FeatureTracker::detect(const cv::Mat &image) {
  cv::imshow("image", image);
  return 0;
}

// int FeatureTracker::addTrack(const Feature &f1, const Feature &f2) {
//  return 0;
// }
//
// int FeatureTracker::removeTrack(const TrackID &track_id, const bool
// lost=false) {
//  return 0;
// }
//
// int FeatureTracker::updateTrack(const TrackID &track_id, const Feature &f) {
//  return 0;
// }

} // namespace gvio
