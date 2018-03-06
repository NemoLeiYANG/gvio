#include "gvio/gimbal/calibration/aprilgrid.hpp"

namespace gvio {

AprilGrid::AprilGrid() {}
AprilGrid::~AprilGrid() {}

int AprilGrid::extractCorners(const cv::Mat &image) {
  assert(this->detector.thisTagFamily.blackborder == 2);
  // If the above fails, it means you have not modified the AprilTag library so
  // that the TagFamily.blackborder == 2, this is required else you will fail
  // to detect the AprilGrid. It is worth noting that changing
  // TagFamily.blackborder == 2, makes it incapable of detecting normal
  // AprilTags.

  // Convert image to gray-scale
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);

  // Extract corners
  std::vector<AprilTags::TagDetection> detections;
  detections = this->detector.extractTags(image_gray);

  // Make an RGB version of the input image
  cv::Mat image_rgb(image_gray.size(), CV_8UC3);
  image_rgb = image.clone();
  cv::cvtColor(image_gray, image_rgb, CV_GRAY2RGB);

  // Iterate through detections
  for (auto &det : detections) {
    const std::pair<float, float> p[4] = {det.p[0],
                                          det.p[1],
                                          det.p[2],
                                          det.p[3]};
    std::cout << p[0].first << " " << p[0].second << std::endl;
    std::cout << p[1].first << " " << p[1].second << std::endl;
    std::cout << p[2].first << " " << p[2].second << std::endl;
    std::cout << p[3].first << " " << p[3].second << std::endl;
    std::cout << std::endl;

    det.draw(image_rgb);
  }
  cv::imshow("Detection", image_rgb);

  return 0;
}

} // namespace gvio
