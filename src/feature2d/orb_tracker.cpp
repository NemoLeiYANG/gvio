#include "gvio/feature2d/orb_tracker.hpp"

namespace gvio {

int ORBTracker::configure(const std::string &config_file) {
  std::string camera_model;
  int image_width = 0;
  int image_height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  // Load config file
  ConfigParser parser;
  // -- Load feature detector settings
  parser.addParam("show_matches", &this->show_matches);
  // -- Load camera model settings
  parser.addParam("camera_model.type", &camera_model, true);
  parser.addParam("camera_model.image_width", &image_width, true);
  parser.addParam("camera_model.image_height", &image_height, true);
  parser.addParam("camera_model.fx", &fx, true);
  parser.addParam("camera_model.fy", &fy, true);
  parser.addParam("camera_model.cx", &cx, true);
  parser.addParam("camera_model.cy", &cy, true);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Load camera model
  if (camera_model == "pinhole") {
    auto *pinhole_model =
        new PinholeModel{image_width, image_height, fx, fy, cx, cy};
    this->camera_model = pinhole_model;
  } else if (camera_model == "none") {
    LOG_INFO("Not loading any camera model!");
  } else {
    LOG_ERROR("Invalid camera model [%s]!", camera_model.c_str());
    return -1;
  }

  this->orb->setFastThreshold(1.0);
  this->orb->setMaxFeatures(100);

  return 0;
}

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
