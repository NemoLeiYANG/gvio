#include "gvio/feature2d/stereo_klt_tracker.hpp"

namespace gvio {

StereoKLTTracker::StereoKLTTracker() {}

StereoKLTTracker::StereoKLTTracker(const CameraProperty &camprop0,
                                   const CameraProperty &camprop1,
                                   const Mat4 &T_cam1_cam0,
                                   const size_t min_track_length,
                                   const size_t max_track_length)
    : camprop0{camprop0}, camprop1{camprop1}, T_cam1_cam0{T_cam1_cam0},
      min_track_length{min_track_length}, max_track_length{max_track_length} {}

StereoKLTTracker::~StereoKLTTracker() {}

std::vector<cv::Point2f> StereoKLTTracker::detect(const cv::Mat &image) {
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
  const Mat3 R_cam1_cam0 = T_cam1_cam0.block(0, 0, 3, 3);
  const auto pts0_ud = camprop0.undistortPoints(pts0, R_cam1_cam0);
  auto pts1 = camprop1.distortPoints(pts0_ud);

  // Perform stereo match
  std::vector<uchar> inliers;
  this->match(cam0_img, cam1_img, pts0, pts1, inliers);
  for (size_t i = 0; i < pts0.size(); i++) {
    if (inliers[i] == 1) {
      this->cam0_pts.emplace_back(pts0[i]);
      this->cam1_pts.emplace_back(pts1[i]);
    }
  }

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

void StereoKLTTracker::track(const cv::Mat &cam0_img,
                             const cv::Mat &cam1_img,
                             std::vector<cv::Point2f> &cam0_pts,
                             std::vector<cv::Point2f> &cam1_pts) {
  // Temporally match cam0 images
  std::vector<cv::Point2f> cam0_pts_new = cam0_pts;
  std::vector<uchar> cam0_pts_mask;
  this->match(this->prev_cam0_img,
              cam0_img,
              cam0_pts,
              cam0_pts_new,
              cam0_pts_mask);

  // Temporally match cam1 images
  std::vector<cv::Point2f> cam1_pts_new = cam1_pts;
  std::vector<uchar> cam1_pts_mask;
  this->match(this->prev_cam1_img,
              cam1_img,
              cam1_pts,
              cam1_pts_new,
              cam1_pts_mask);

  // Stereo match cam0 and cam1 points
  std::vector<uchar> stereo_mask;
  this->match(cam0_img, cam1_img, cam0_pts_new, cam1_pts_new, stereo_mask);

  // Remove outliers
  std::vector<cv::Point2f> pts0;
  std::vector<cv::Point2f> pts1;
  for (size_t i = 0; i < cam0_pts.size(); i++) {
    if (cam0_pts_mask[i] && cam1_pts_mask[i] && stereo_mask[i]) {
      const auto p0 = cam0_pts_new[i];
      const auto p1 = cam1_pts_new[i];
      pts0.emplace_back(p0);
      pts1.emplace_back(p1);
    }
  }

  // Update
  cam0_pts = pts0;
  cam1_pts = pts1;
  this->prev_cam0_img = cam0_img.clone();
  this->prev_cam1_img = cam1_img.clone();

  if (this->show_matches) {
    std::vector<uchar> mask(pts0.size(), 1);
    cv::Mat match = draw_matches(cam0_img, cam1_img, pts0, pts1, mask);
    cv::imshow("Match", match);
    cv::waitKey(1);
  }
}

int StereoKLTTracker::update(const cv::Mat &cam0_img, const cv::Mat &cam1_img) {
  // Initialize feature tracker
  if (this->cam0_pts.size() == 0 || this->cam1_pts.size() == 0) {
    this->cam0_pts.clear();
    this->cam1_pts.clear();
    this->initialize(cam0_img, cam1_img);
    return 0;
  }

  // Track features
  this->counter_frame_id++;
  this->track(cam0_img, cam1_img, this->cam0_pts, this->cam1_pts);

  // Compute the relative rotation between the cam0
  // frame and cam1 frame.
  // const Mat3 R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu;
  // const Vec3 t_cam0_cam1 = R_cam1_imu.t() * (t_cam0_imu-t_cam1_imu);
  // Compute the essential matrix.
  // const Mat3 E = skew(t_cam0_cam1) * R_cam0_cam1;

  // // Visualize tracks
  // if (this->show_tracks) {
  //   const cv::Mat cam0_tracks = draw_tracks(cam0_img, this->cam0_pts);
  //   const cv::Mat cam1_tracks = draw_tracks(cam1_img, this->cam1_pts);
  //
  //   cv::Mat tracks;
  //   cv::vconcat(cam0_tracks, cam1_tracks, tracks);
  //   cv::imshow("Tracks", tracks);
  //   cv::waitKey(1);
  // }

  return 0;
}

} // namespace gvio
