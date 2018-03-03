#include "gvio/gimbal/calibration/aprilgrid.hpp"

namespace gvio {

AprilGrid::AprilGrid() {}
AprilGrid::~AprilGrid() {}

int AprilGrid::extractCorners(const cv::Mat &image) {
  // Extract corners
  std::vector<AprilTags::TagDetection> detections;

  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  detections = this->detector.extractTags(image_gray);

  // Iterate through detections
  // cv::Mat image_rgb(image_gray.size(), CV_8UC3);
  cv::Mat image_rgb;
  image_rgb = image.clone();
  // cv::cvtColor(image_gray, image_rgb, CV_GRAY2RGB);

  for (auto &det : detections) {
    // const std::pair<float, float> p[4] = {det.p[0],
    //                                       det.p[1],
    //                                       det.p[2],
    //                                       det.p[3]};

    det.draw(image_rgb);
  }
  cv::imshow("Detection", image_rgb);

  return 0;
}

} // namespace gvio
