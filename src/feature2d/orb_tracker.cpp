#include "gvio/feature2d/orb_tracker.hpp"

namespace gvio {

ORBTracker::ORBTracker() {}

ORBTracker::ORBTracker(CameraProperty *camera_property)
    : FeatureTracker{camera_property} {}

ORBTracker::ORBTracker(CameraProperty *camera_property,
                       const size_t min_track_length,
                       const size_t max_track_length)
    : FeatureTracker{camera_property, min_track_length, max_track_length} {}

ORBTracker::~ORBTracker() {}

int ORBTracker::detect(const cv::Mat &image, Features &features) {
  // Feature descriptor extraction
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat mask;
  cv::Mat descriptors;
  this->orb->detectAndCompute(image, mask, keypoints, descriptors);

  // Update counters
  this->counter_frame_id += 1;

  // Create features
  for (int i = 0; i < descriptors.rows; i++) {
    features.emplace_back(keypoints[i], descriptors.row(i));
  }

  return 0;
}

} // namespace gvio
