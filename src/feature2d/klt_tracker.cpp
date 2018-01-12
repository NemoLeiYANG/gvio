#include "gvio/feature2d/klt_tracker.hpp"

namespace gvio {

int KLTTracker::configure(const std::string &config_file) {
  std::string camera_model;
  int image_width = 0;
  int image_height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  // Load config file
  ConfigParser parser;
  parser.addParam("nb_max_corners", &this->nb_max_corners);
  parser.addParam("quality_level", &this->quality_level);
  parser.addParam("min_distance", &this->min_distance);
  parser.addParam("show_matches", &this->show_matches);
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

  return 0;
}

std::vector<FeatureTrack> KLTTracker::getLostTracks() {
  // Get lost tracks
  std::vector<FeatureTrack> tracks;
  this->features.removeLostTracks(tracks);
  if (this->camera_model == nullptr) {
    return tracks;
  }

  // Transform keypoints
  for (auto &track : tracks) {
    for (auto &feature : track.track) {
      // Convert pixel coordinates to image coordinates
      const Vec2 pt = this->camera_model->pixel2image(feature.kp.pt);
      feature.kp.pt.x = pt(0);
      feature.kp.pt.y = pt(1);
    }
  }

  return tracks;
}

int KLTTracker::initialize(const cv::Mat &img_cur) {
  DEBUG("Initialize feature tracker!");
  img_cur.copyTo(this->img_ref);
  this->counter_frame_id++;
  return this->detect(img_cur, this->features.fea_ref);
}

int KLTTracker::detect(const cv::Mat &image, Features &features) {
  // Convert image to gray scale
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, CV_BGR2GRAY);

  // Feature detection
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(gray_image,
                          corners,
                          this->nb_max_corners,
                          this->quality_level,
                          this->min_distance);

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

  // Add, update or remove feature tracks
  Features keeping;
  int index = 0;

  for (auto s : status) {
    auto f0 = features[index];

    // Feature tracked
    if (s == 1) {
      auto f1 = Feature(p1[index]);
      if (f0.track_id == -1) {
        this->features.addTrack(this->counter_frame_id, f0, f1);
      } else {
        this->features.updateTrack(this->counter_frame_id, f0.track_id, f1);
      }
      keeping.push_back(f1);

      // Feature lost
    } else {
      this->features.removeTrack(f0.track_id, true);
    }

    index++;
  }
  this->features.fea_ref = keeping;

  return 0;
}

int KLTTracker::update(const cv::Mat &img_cur) {
  // Keep track of current image
  img_cur.copyTo(this->img_cur);

  // Initialize feature tracker
  if (this->features.fea_ref.size() == 0) {
    this->initialize(img_cur);
    return 0;
  }

  // Detect
  if (this->features.fea_ref.size() < 200) {
    if (this->detect(img_cur, this->features.fea_ref) != 0) {
      return -1;
    }
  }
  this->counter_frame_id++;

  // Match features
  if (this->track(this->features.fea_ref) != 0) {
    return -2;
  }

  // Update
  img_cur.copyTo(this->img_ref);

  return 0;
}

} // namespace gvio
