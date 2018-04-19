#include "gvio/feature2d/draw.hpp"

namespace gvio {

cv::Mat draw_tracks(const cv::Mat &img_cur,
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

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::Point2f> k0,
                     const std::vector<cv::Point2f> k1,
                     const std::vector<uchar> &status) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < status.size(); i++) {
    if (status[i]) {
      cv::Point2f p0 = k0[i];
      cv::Point2f p1 = k1[i];

      // Point 1
      p1.y += img0.rows;

      // Draw circle and line
      cv::circle(match_img, p0, 2, cv::Scalar(0, 255, 0), -1);
      cv::circle(match_img, p1, 2, cv::Scalar(0, 255, 0), -1);
      cv::line(match_img, p0, p1, cv::Scalar(0, 255, 0));
    }
  }

  return match_img;
}

cv::Mat draw_matches(const cv::Mat &img0,
                     const cv::Mat &img1,
                     const std::vector<cv::KeyPoint> k0,
                     const std::vector<cv::KeyPoint> k1,
                     const std::vector<cv::DMatch> &matches) {
  cv::Mat match_img;

  // Stack current and previous image vertically
  cv::vconcat(img0, img1, match_img);

  // Draw matches
  for (size_t i = 0; i < matches.size(); i++) {
    const int k0_idx = matches[i].queryIdx;
    const int k1_idx = matches[i].trainIdx;
    cv::KeyPoint p0 = k0[k0_idx];
    cv::KeyPoint p1 = k1[k1_idx];

    // Point 1
    p1.pt.y += img0.rows;

    // Draw circle and line
    cv::circle(match_img, p0.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(match_img, p1.pt, 2, cv::Scalar(0, 255, 0), -1);
    cv::line(match_img, p0.pt, p1.pt, cv::Scalar(0, 255, 0));
  }

  return match_img;
}

cv::Mat draw_features(const cv::Mat &image, const Features features) {
  cv::Mat fea_img;

  image.copyTo(fea_img);
  for (auto f : features) {
    cv::circle(fea_img, f.kp.pt, 2, cv::Scalar(0, 255, 0), -1);
  }

  return fea_img;
}

} // namespace gvio
