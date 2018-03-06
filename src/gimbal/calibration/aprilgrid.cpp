#include "gvio/gimbal/calibration/aprilgrid.hpp"

namespace gvio {

// TODO: estimate transform from target to aprilgrid

AprilGrid::AprilGrid() {}

AprilGrid::AprilGrid(const int tag_rows,
                     const int tag_cols,
                     const double tag_size,
                     const double tag_spacing)
    : tag_rows{tag_rows}, tag_cols{tag_cols}, tag_size{tag_size},
      tag_spacing{tag_spacing} {

  // Construct an AprilGrid calibration target
  //
  // tag_rows:    number of tags in y-dir
  // tag_cols:    number of tags in x-dir
  // tag_size:    size of a tag [m]
  // tag_spacing: space between tags in [m] (= tag_spacing * tag_size)
  //
  // Corner ordering in AprilGrid.object_points:
  //
  //   12-----13  14-----15
  //   | TAG 3 |  | TAG 4 |
  //   8-------9  10-----11
  //   4-------5  6-------7
  // y | TAG 1 |  | TAG 2 |
  // ^ 0-------1  2-------3
  // |-->x
  const int rows = tag_rows * 2;
  const int cols = tag_cols * 2;
  this->object_points.resize(rows * cols, 3);

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      // clang-format off
      const double x = (int) (c / 2) * (1 + tag_spacing) * tag_size + (c % 2) * tag_size;
      const double y = (int) (r / 2) * (1 + tag_spacing) * tag_size + (r % 2) * tag_size;
      const double z = 0.0;
      const Vec3 point{x, y, z};
      this->object_points.row(r * cols + c) = point.transpose();
      // clang-format on
    }
  }

  std::cout << this->object_points << std::endl;
}

AprilGrid::~AprilGrid() {}

int AprilGrid::detect(cv::Mat &image) {
  // Convert image to gray-scale
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);

  // Extract corners
  std::vector<AprilTags::TagDetection> detections;
  detections = this->detector.extractTags(image_gray);
  if (detections.size() == 0) {
    return -1;
  }

  // Make an RGB version of the input image
  cv::Mat image_rgb(image_gray.size(), CV_8UC3);
  image_rgb = image.clone();
  cv::cvtColor(image_gray, image_rgb, CV_GRAY2RGB);

  // Iterate through detections
  for (auto &det : detections) {
    det.draw(image_rgb);
  }
  image = image_rgb.clone();

  return 0;
}

int AprilGrid::solvePnP(const std::map<int, std::vector<Vec2>> &tags) {

  return 0;
}

int AprilGrid::extractTags(cv::Mat &image,
                           std::map<int, std::vector<Vec2>> &tags) {
  assert(this->detector.thisTagFamily.blackBorder == 2);
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
    const Vec2 p0{det.p[0].first, det.p[0].second};
    const Vec2 p1{det.p[1].first, det.p[1].second};
    const Vec2 p2{det.p[2].first, det.p[2].second};
    const Vec2 p3{det.p[3].first, det.p[3].second};
    const std::vector<Vec2> corners = {p0, p1, p2, p3};
    tags.emplace(det.id, corners);

    det.draw(image_rgb);
  }
  image = image_rgb.clone();

  return 0;
}

} // namespace gvio
