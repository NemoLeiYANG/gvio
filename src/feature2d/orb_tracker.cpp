#include "gvio/feature2d/orb_tracker.hpp"

namespace gvio {

ORBTracker::ORBTracker() {}

ORBTracker::ORBTracker(const CameraProperty &camera_property)
    : FeatureTracker{camera_property} {}

ORBTracker::ORBTracker(const CameraProperty &camera_property,
                       const size_t min_track_length,
                       const size_t max_track_length)
    : FeatureTracker{camera_property, min_track_length, max_track_length} {}

ORBTracker::~ORBTracker() {}

int ORBTracker::detect(const cv::Mat &image, Features &features) {
  // // Feature descriptor extraction
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat mask;
  cv::Mat descriptors;
  // this->orb->detectAndCompute(image, mask, keypoints, descriptors);

  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  keypoints = grid_fast(image_gray, 1000, 5, 5, 40.0);
  this->orb->compute(image, keypoints, descriptors);

  // for (auto kp : keypoints) {
  //   std::cout << kp.response << std::endl;
  // }
  // exit(0);

  // Update counters
  this->counter_frame_id += 1;

  // Create features
  for (int i = 0; i < descriptors.rows; i++) {
    features.emplace_back(keypoints[i], descriptors.row(i));
  }

  return 0;
}

} // namespace gvio
