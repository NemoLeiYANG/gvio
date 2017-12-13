#include "gvio/feature2d/orb_tracker.hpp"

namespace gvio {

int ORBTracker::detect(const cv::Mat &image,
                       std::vector<Feature> &features) {
  // Feature descriptor extraction
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat mask;
  cv::Mat descriptors;
  this->orb->detectAndCompute(image, mask, keypoints, descriptors);

  // Update counters
  this->counter_frame_id++;

  // Create features
  // for (int i = 0; i < descriptors.rows; i++) {
  for (int i = 0; i < 200; i++) {
    features.emplace_back(keypoints[i], descriptors.row(i));
  }

  return 0;
}

} // namespace gvio
