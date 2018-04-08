#include "gvio/feature2d/klt_tracker.hpp"

namespace gvio {

// int KLTTracker::configure(const std::string &config_file) {
//   std::string camera_model;
//   int image_width = 0;
//   int image_height = 0;
//   double fx = 0.0;
//   double fy = 0.0;
//   double cx = 0.0;
//   double cy = 0.0;
//
//   // Load config file
//   ConfigParser parser;
//   // -- Load feature detector settings
//   parser.addParam("max_corners", &this->max_corners);
//   parser.addParam("quality_level", &this->quality_level);
//   parser.addParam("min_distance", &this->min_distance);
//   parser.addParam("show_matches", &this->show_matches);
//   // -- Load camera model settings
//   parser.addParam("camera_model.type", &camera_model, true);
//   parser.addParam("camera_model.image_width", &image_width, true);
//   parser.addParam("camera_model.image_height", &image_height, true);
//   parser.addParam("camera_model.fx", &fx, true);
//   parser.addParam("camera_model.fy", &fy, true);
//   parser.addParam("camera_model.cx", &cx, true);
//   parser.addParam("camera_model.cy", &cy, true);
//   if (parser.load(config_file) != 0) {
//     LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
//     return -1;
//   }
//
//   // Load camera model
//   if (camera_model == "pinhole") {
//     this->camera_model =
//         new PinholeModel{image_width, image_height, fx, fy, cx, cy};
//   } else {
//     LOG_ERROR("Invalid camera model [%s]!", camera_model.c_str());
//     return -1;
//   }
//
//   return 0;
// }

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
  img_cur.copyTo(this->img_ref);
  this->counter_frame_id++;
  this->image_width = img_cur.cols;
  this->image_height = img_cur.rows;
  return this->detect(img_cur, this->fea_ref);
}

int KLTTracker::detect(const cv::Mat &image, Features &features) {
  // Convert image to gray scale
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, CV_BGR2GRAY);

  // Feature detection
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(gray_image,
                          corners,
                          this->max_corners,
                          this->quality_level,
                          this->min_distance);

  // Create features
  features.clear();
  for (auto corner : corners) {
    features.emplace_back(corner);
  }

  return 0;
}

int KLTTracker::processTrack(const uchar status, Feature &fref, Feature &fcur) {
  // Lost - Remove feature track
  if (status == 0 && fref.track_id != -1) {
    this->features.removeTrack(fref.track_id, true);
    return 0;
  }

  // Tracked - Add or update feature track
  if (fref.track_id == -1) {
    this->features.addTrack(this->counter_frame_id, fref, fcur);
  } else {
    this->features.updateTrack(this->counter_frame_id, fref.track_id, fcur);
  }
  return 1;
}

int KLTTracker::track(const cv::Mat &img_ref,
                      const cv::Mat &img_cur,
                      const Features &fea_ref,
                      Features &tracked) {
  // Convert list of features to list of cv::Point2f
  std::vector<cv::Point2f> p0;
  for (auto f : fea_ref) {
    p0.push_back(f.kp.pt);
  }

  // Convert input images to gray scale
  cv::Mat gray_img_ref, gray_img_cur;
  cv::cvtColor(img_ref, gray_img_ref, CV_BGR2GRAY);
  cv::cvtColor(img_cur, gray_img_cur, CV_BGR2GRAY);

  // Track features with KLT
  std::vector<cv::Point2f> p1;
  std::vector<uchar> flow_mask;
  std::vector<float> err;
  cv::Size win_size(21, 21);
  const int pyr_levels = 3;
  cv::calcOpticalFlowPyrLK(gray_img_ref, // Reference image
                           gray_img_cur, // Current image
                           p0,           // Input points
                           p1,           // Output points
                           flow_mask,    // Tracking status
                           err,          // Tracking error
                           win_size,     // Window size
                           pyr_levels);  // Pyramid levels

  // RANSAC
  std::vector<uchar> ransac_mask;
  cv::findFundamentalMat(p0, p1, cv::FM_RANSAC, 0, 0.9999, ransac_mask);

  // Add, update or remove feature tracks
  std::vector<uchar> final_mask;
  for (size_t i = 0; i < flow_mask.size(); i++) {
    auto fref = fea_ref[i];
    auto fcur = Feature(p1[i]);
    const uchar status = (flow_mask[i] && ransac_mask[i]) ? 1 : 0;

    if (this->processTrack(status, fref, fcur)) {
      tracked.push_back(fcur);
    }

    final_mask.push_back(status);
  }

  // Show matches
  if (this->show_matches) {
    cv::Mat matches_img = draw_tracks(img_cur, p0, p1, final_mask);
    cv::imshow("Matches", img_cur);
    cv::waitKey(1);
  }

  return 0;
}

int KLTTracker::replenishFeatures(const cv::Mat &image, Features &features) {
  // Pre-check
  const int replenish_size = this->max_corners - features.size();
  if (replenish_size <= 0) {
    return 0;
  }

  // Build a grid denoting where existing keypoints already are
  MatX pt_grid = zeros(this->image_height, this->image_width);
  for (auto f : features) {
    const int px = int(f.kp.pt.x);
    const int py = int(f.kp.pt.y);
    if (px >= this->image_width || px <= 0) {
      continue;
    } else if (py >= this->image_height || py <= 0) {
      continue;
    }
    pt_grid(py, px) = 1;
  }

  // Detect new features
  Features fea_new;
  this->detect(image, fea_new);
  for (auto f : fea_new) {
    const int px = int(f.kp.pt.x);
    const int py = int(f.kp.pt.y);
    if (pt_grid(py, px) == 0) {
      features.push_back(f);
    }
  }

  return 0;
}

int KLTTracker::update(const cv::Mat &img_cur) {
  // Initialize feature tracker
  if (this->fea_ref.size() == 0) {
    this->initialize(img_cur);
    return 0;
  }

  // Track features
  this->counter_frame_id++;
  Features tracked;
  int retval = this->track(this->img_ref, img_cur, this->fea_ref, tracked);
  if (retval != 0) {
    return -1;
  }
  this->fea_ref = tracked;

  // Replenish number of features
  if (this->replenishFeatures(img_cur, this->fea_ref) != 0) {
    return -1;
  }

  // Update
  img_cur.copyTo(this->img_ref);

  return 0;
}

int KLTTracker::update2(const cv::Mat &img_cur, const long ts) {
  UNUSED(ts);
  return this->update(img_cur);
}

} // namespace gvio
