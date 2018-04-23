#include "gvio/feature2d/stereo_klt_tracker.hpp"

namespace gvio {

StereoKLTTracker::StereoKLTTracker() {}

StereoKLTTracker::StereoKLTTracker(const CameraProperty &camprop0,
                                   const CameraProperty &camprop1,
                                   const Mat4 &T_cam1_cam0,
                                   const size_t min_track_length,
                                   const size_t max_track_length)
    : camprop0{camprop0}, camprop1{camprop1}, T_cam1_cam0{T_cam1_cam0},
      features{min_track_length, max_track_length} {}

StereoKLTTracker::~StereoKLTTracker() {}

std::vector<cv::Point2f> StereoKLTTracker::detect(const cv::Mat &image) {
  // Keep track of image width and height
  if (this->image_width == 0) {
    this->image_width = image.cols;
  }
  if (this->image_height == 0) {
    this->image_height = image.rows;
  }

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

  // std::vector<cv::KeyPoint> keypoints;
  // cv::FAST(gray_image, keypoints, 30, false);
  // for (auto kp : keypoints) {
  //   corners.emplace_back(kp.pt);
  // }

  return corners;
}

void StereoKLTTracker::match(const cv::Mat &img0,
                             const cv::Mat &img1,
                             std::vector<cv::Point2f> &pts0,
                             std::vector<cv::Point2f> &pts1,
                             std::vector<uchar> &mask) {
  // Perform optical flow matching between cam0 and cam1 images
  std::vector<uchar> flow_mask;
  std::vector<float> err;
  cv::Size win_size(21, 21);
  const int pyr_levels = this->pyramid_levels;
  auto term_crit =
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                       this->max_iteration,
                       this->track_precision);
  auto flags = cv::OPTFLOW_USE_INITIAL_FLOW;
  cv::calcOpticalFlowPyrLK(img0,       // First image
                           img1,       // Second image
                           pts0,       // Input points
                           pts1,       // Output points
                           flow_mask,  // Tracking status
                           err,        // Tracking error
                           win_size,   // Window size
                           pyr_levels, // Pyramid levels
                           term_crit,  // Termination criteria
                           flags);     // Flags

  // RANSAC
  std::vector<uchar> ransac_mask;
  cv::findFundamentalMat(pts0, pts1, cv::FM_RANSAC, 1, 0.9999, ransac_mask);

  // Create final mask
  for (size_t i = 0; i < pts0.size(); i++) {
    const uchar status = (flow_mask[i] && ransac_mask[i]) ? 1 : 0;
    mask.push_back(status);
  }
}

int StereoKLTTracker::initialize(const cv::Mat &cam0_img,
                                 const cv::Mat &cam1_img) {
  // Detect features in cam0
  std::vector<cv::Point2f> pts0 = this->detect(cam0_img);
  if (pts0.size() == 0) {
    LOG_ERROR("Failed to detect any features !");
    return -1;
  }

  // Project keypoints from cam0 to cam1 to form k1
  const Mat3 R_cam1_cam0 = this->T_cam1_cam0.block(0, 0, 3, 3);
  const auto pts0_ud = this->camprop0.undistortPoints(pts0, R_cam1_cam0);
  auto pts1 = this->camprop1.distortPoints(pts0_ud);

  // Perform stereo match
  std::vector<uchar> inliers;
  this->match(cam0_img, cam1_img, pts0, pts1, inliers);
  for (size_t i = 0; i < pts0.size(); i++) {
    if (inliers[i] == 1) {
      this->cam0_pts.emplace_back(pts0[i]);
      this->cam1_pts.emplace_back(pts1[i]);
    }
  }

  // Initialize track_ids
  this->track_ids = std::vector<int>(this->cam0_pts.size(), -1);

  // Make sure we're tracking someting
  if (this->cam0_pts.size() == 0) {
    LOG_ERROR("Failed stereo matching!");
    return -1;
  }

  // Initialize counters and previous cam0 and cam1 images
  this->counter_frame_id++;
  this->prev_cam0_img = cam0_img.clone();
  this->prev_cam1_img = cam1_img.clone();

  return 0;
}

int StereoKLTTracker::updateTrack(const int index,
                                  const bool is_inlier,
                                  const cv::Point2f &pt0_ref,
                                  const cv::Point2f &pt0_cur,
                                  const cv::Point2f &pt1_ref,
                                  const cv::Point2f &pt1_cur) {
  auto cam0_f0 = Feature(pt0_ref);
  auto cam0_f1 = Feature(pt0_cur);
  auto cam1_f0 = Feature(pt1_ref);
  auto cam1_f1 = Feature(pt1_cur);
  int track_id = this->track_ids[index];

  // Remove feature track
  if (is_inlier == false) {
    this->features.removeTrack(track_id);
    return -1;
  }

  if (track_id == -1) {
    // Add new feature track
    this->features.addStereoTrack(this->counter_frame_id,
                                  cam0_f0,
                                  cam0_f1,
                                  cam1_f0,
                                  cam1_f1);
    track_id = cam0_f0.track_id;

  } else {
    // Update feature track
    int too_old = this->features.updateStereoTrack(this->counter_frame_id,
                                                   track_id,
                                                   cam0_f1,
                                                   cam1_f1);
    if (too_old == 1) {
      return -1;
    }
  }

  return track_id;
}

void StereoKLTTracker::trackFeatures(const cv::Mat &cam0_img,
                                     const cv::Mat &cam1_img) {
  // Make a copy of feature points currently tracking
  std::vector<cv::Point2f> pts0_ref = this->cam0_pts;
  std::vector<cv::Point2f> pts1_ref = this->cam1_pts;
  assert(pts0_ref.size() == pts1_ref.size());
  assert(pts0_ref.size() > 0);

  // Temporally match cam0 images
  std::vector<cv::Point2f> pts0_cur = pts0_ref;
  std::vector<uchar> t0_mask;
  this->match(this->prev_cam0_img, cam0_img, pts0_ref, pts0_cur, t0_mask);

  // Temporally match cam1 images
  std::vector<cv::Point2f> pts1_cur = pts1_ref;
  std::vector<uchar> t1_mask;
  this->match(this->prev_cam1_img, cam1_img, pts1_ref, pts1_cur, t1_mask);

  // Stereo match cam0 and cam1 points
  std::vector<uchar> s_mask;
  this->match(cam0_img, cam1_img, pts0_cur, pts1_cur, s_mask);

  // Remove outliers
  std::vector<cv::Point2f> pts0_inliers;
  std::vector<cv::Point2f> pts1_inliers;
  std::vector<int> tracks_tracking;

  for (size_t i = 0; i < pts0_ref.size(); i++) {
    bool is_inlier = (t0_mask[i] && t1_mask[i] && s_mask[i]) ? true : false;
    int track_id = this->updateTrack(i,
                                     is_inlier,
                                     pts0_ref[i],
                                     pts0_cur[i],
                                     pts1_ref[i],
                                     pts1_cur[i]);
    if (is_inlier && track_id != -1) {
      pts0_inliers.push_back(pts0_cur[i]);
      pts1_inliers.push_back(pts1_cur[i]);
      tracks_tracking.push_back(track_id);
    }
  }

  // Update
  this->track_ids = tracks_tracking;
  this->cam0_pts = pts0_inliers;
  this->cam1_pts = pts1_inliers;
  this->prev_cam0_img = cam0_img.clone();
  this->prev_cam1_img = cam1_img.clone();

  if (this->show_matches) {
    std::vector<uchar> mask(pts0_inliers.size(), 1);
    cv::Mat match =
        draw_matches(cam0_img, cam1_img, pts0_inliers, pts1_inliers, mask);
    cv::imshow("Match", match);
    cv::waitKey(1);
  }
}

void StereoKLTTracker::replenishFeatures(const cv::Mat &image) {
  // Pre-check
  const int replenish_size = this->max_corners - this->cam0_pts.size();
  if (replenish_size <= 0) {
    return;
  }

  // Build a grid denoting where existing keypoints already are
  assert(this->image_width > 0);
  assert(this->image_height > 0);
  MatX pt0_grid = zeros(this->image_height, this->image_width);
  for (auto p : this->cam0_pts) {
    const int px = int(p.x);
    const int py = int(p.y);
    if (px >= this->image_width || px <= 0) {
      continue;
    } else if (py >= this->image_height || py <= 0) {
      continue;
    }
    pt0_grid(py, px) = 1;
  }

  // Detect new features
  auto corners = this->detect(image);
  std::vector<cv::Point2f> pts0_new;
  for (auto p : corners) {
    const int px = int(p.x);
    const int py = int(p.y);
    if (pt0_grid(py, px) == 0 && pts0_new.size() < (size_t) replenish_size) {
      pts0_new.emplace_back(px, py);
    }
  }

  // Project new keypoints from cam0 to cam1
  const Mat3 R_cam1_cam0 = this->T_cam1_cam0.block(0, 0, 3, 3);
  const auto pts0_ud = this->camprop0.undistortPoints(pts0_new, R_cam1_cam0);
  auto pts1_new = this->camprop1.distortPoints(pts0_ud);

  // Append new keypoints to cam0_pts and cam1_pts
  const auto new_size = std::distance(pts0_new.begin(), pts0_new.end());
  const std::vector<int> ids_new(new_size, -1);

  this->cam0_pts.reserve(this->cam0_pts.size() + new_size);
  this->cam1_pts.reserve(this->cam1_pts.size() + new_size);
  this->track_ids.reserve(this->track_ids.size() + new_size);

  this->cam0_pts.insert(this->cam0_pts.end(), pts0_new.begin(), pts0_new.end());
  this->cam1_pts.insert(this->cam1_pts.end(), pts1_new.begin(), pts1_new.end());
  this->track_ids.insert(this->track_ids.end(), ids_new.begin(), ids_new.end());
}

int StereoKLTTracker::update(const cv::Mat &cam0_img, const cv::Mat &cam1_img) {
  // Initialize feature tracker
  if (this->cam0_pts.size() == 0) {
    this->cam0_pts.clear();
    this->cam1_pts.clear();
    this->initialize(cam0_img, cam1_img);
    return 0;
  }

  // Track features
  this->counter_frame_id++;
  this->trackFeatures(cam0_img, cam1_img);

  // Replenish features
  this->replenishFeatures(cam0_img);

  return 0;
}

int StereoKLTTracker::update2(const cv::Mat &cam0_img,
                              const cv::Mat &cam1_img,
                              long ts) {
  UNUSED(ts);
  return this->update(cam0_img, cam1_img);
}

std::vector<FeatureTrack> StereoKLTTracker::getLostTracks() {
  // Get lost tracks
  std::vector<FeatureTrack> tracks;
  this->features.removeLostTracks(tracks);

  // Transform keypoints
  for (auto &track : tracks) {

    // Undistort points and convert pixel coordinates to image coordinates
    for (size_t i = 0; i < track.trackedLength(); i++) {
      auto &f0 = track.track0[i];
      auto &f1 = track.track1[i];
      f0.kp.pt = this->camprop0.undistortPoint(f0.kp.pt);
      f1.kp.pt = this->camprop1.undistortPoint(f1.kp.pt);
    }
  }

  return tracks;
}

} // namespace gvio
