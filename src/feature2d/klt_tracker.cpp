#include "gvio/feature2d/klt_tracker.hpp"

namespace gvio {

int KLTTracker::detect(const cv::Mat &image,
                       std::vector<Feature> &features) {
  // Convert image to gray scale
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, CV_BGR2GRAY);

  // Feature detection
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(gray_image, corners, 1000, 0.01, 10);

  // Update counters
  this->counter_frame_id++;

  // Create features
  features.clear();
  for (auto corner : corners) {
    features.emplace_back(corner);
  }

  return 0;
}

cv::Mat KLTTracker::drawTracks(const cv::Mat &img_cur,
                               const std::vector<cv::Point2f> p0,
                               const std::vector<cv::Point2f> p1,
                               const std::vector<uchar> &status) {
  // Draw tracks
  for (size_t i = 0; i < status.size(); i++) {
    // Check if point was lost
    if (status[i] == 0) {
      continue;
    }

    // Draw circle and line
    cv::circle(img_cur, p0[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::circle(img_cur, p1[i], 1, cv::Scalar(0, 255, 0), -1);
    cv::line(img_cur, p0[i], p1[i], cv::Scalar(0, 255, 0));
  }

  return img_cur;
}

int KLTTracker::track(const std::vector<Feature> &features) {
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
  cv::Size win_size(40, 40);
  cv::calcOpticalFlowPyrLK(gray_img_ref,  // Reference image
                           gray_img_cur,  // Current image
                           p0,            // Input points
                           p1,            // Output points
                           status,        // Tracking status
                           err,           // Tracking error
                           win_size);     // Window size

  // Show matches
  if (this->show_matches) {
    cv::Mat matches_img = this->drawTracks(this->img_cur, p0, p1, status);
    cv::imshow("Matches", matches_img);
  }

  // Update or remove feature track
  int index = 0;
  std::vector<Feature> keeping;
  std::map<Feature, bool, CompareFeatureByKeyPoint> tracking;

  for (auto s : status) {
    auto f0 = features[index];

    if (s == 1) {
      auto f1 = Feature(p1[index]);

      if (f0.track_id == -1) {
        this->addTrack(f0, f1);
      } else {
        this->updateTrack(f0.track_id, f1);
      }

      this->fea_ref.push_back(f1);
      keeping.push_back(f1);
      tracking.insert({f1, true});

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

  // Match features
  if (this->track(this->fea_ref) != 0) {
    return -2;
  }

  // Update
  img_cur.copyTo(this->img_ref);

  return 0;
}

} // namespace gvio
