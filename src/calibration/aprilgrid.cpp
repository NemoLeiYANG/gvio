#include "gvio/calibration/aprilgrid.hpp"

namespace gvio {

AprilGrid::AprilGrid() {}

AprilGrid::AprilGrid(const int tag_rows,
                     const int tag_cols,
                     const double tag_size,
                     const double tag_spacing)
    : tag_rows{tag_rows}, tag_cols{tag_cols}, tag_size{tag_size},
      tag_spacing{tag_spacing} {

  // Construct an AprilGrid object points
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
  const int corner_rows = tag_rows * 2;
  const int corner_cols = tag_cols * 2;
  this->grid_points.resize(corner_rows * corner_cols, 3);

  for (int r = 0; r < corner_rows; r++) {
    for (int c = 0; c < corner_cols; c++) {
      // clang-format off
      const double x = (int) (c / 2) * (1 + tag_spacing) * tag_size + (c % 2) * tag_size;
      const double y = (int) (r / 2) * (1 + tag_spacing) * tag_size + (r % 2) * tag_size;
      const double z = 0.0;
      const Vec3 point{x, y, z};
      this->grid_points.row(r * corner_cols + c) = point.transpose();
      // clang-format on
    }
  }

  // Construct tag coordinates:
  //
  //   12-----13  14-----15
  //   | (0,1) |  | (1,1) |
  //   8-------9  10-----11
  //   4-------5  6-------7
  // y | (0,0) |  | (1,0) |
  // ^ 0-------1  2-------3
  // |-->x
  int i = 0;
  for (int r = 0; r < this->tag_rows; r++) {
    for (int c = 0; c < this->tag_cols; c++) {
      this->tag_coordinates[i] = {c, r};
      i++;
    }
  }
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

int AprilGrid::extractTags(cv::Mat &image,
                           std::map<int, std::vector<cv::Point2f>> &tags) {
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
    const cv::Point2f p0{det.p[0].first, det.p[0].second};
    const cv::Point2f p1{det.p[1].first, det.p[1].second};
    const cv::Point2f p2{det.p[2].first, det.p[2].second};
    const cv::Point2f p3{det.p[3].first, det.p[3].second};
    const std::vector<cv::Point2f> corners = {p0, p1, p2, p3};
    tags.emplace(det.id, corners);

    // det.draw(image_rgb);
  }
  image = image_rgb.clone();

  return 0;
}

std::vector<cv::Point3f> AprilGrid::formObjectPoints(
    const std::map<int, std::vector<cv::Point2f>> &tags) {
  // Preprocess corners and see which ones are observed
  const int rows = this->tag_rows * 2;
  const int cols = this->tag_cols * 2;
  MatX corners = zeros(rows * cols, 3);

  int i = 0;
  for (auto &tag : tags) {
    // Get tag coordinate
    const int tag_id = tag.first;
    auto tag_pos = this->tag_coordinates[tag_id];
    const int x = tag_pos.first;
    const int y = tag_pos.second;

    // From tag coordinate work out the corner indicies
    const int cols = this->tag_cols * 2;
    const int bottom_left = (cols * 2) * y + x * 2;
    const int bottom_right = (cols * 2) * y + (x * 2) + 1;
    const int top_right = ((cols * 2) * y + cols) + (x * 2) + 1;
    const int top_left = ((cols * 2) * y + cols) + (x * 2);

    // std::cout << "tag id: " << tag_id << std::endl;
    // std::cout << "tag coordinate: (";
    // std::cout << tag_pos.first << ", ";
    // std::cout << tag_pos.second << ")" << std::endl;
    // std::cout << "bottom_left: " << bottom_left << std::endl;
    // std::cout << "bottom_right: " << bottom_right << std::endl;
    // std::cout << "top_right: " << top_right << std::endl;
    // std::cout << "top_left: " << top_left << std::endl;
    // std::cout << std::endl;

    // From the corner indicies get the corner grid points
    const Vec3 p0 = this->grid_points.row(bottom_left).transpose();
    const Vec3 p1 = this->grid_points.row(bottom_right).transpose();
    const Vec3 p2 = this->grid_points.row(top_right).transpose();
    const Vec3 p3 = this->grid_points.row(top_left).transpose();

    corners.block(i, 0, 1, 3) = p0.transpose();
    corners.block(i + 1, 0, 1, 3) = p1.transpose();
    corners.block(i + 2, 0, 1, 3) = p2.transpose();
    corners.block(i + 3, 0, 1, 3) = p3.transpose();
    i += 4;

    // corners.block(bottom_left, 0, 1, 3) = p0.transpose();
    // corners.block(bottom_right, 0, 1, 3) = p1.transpose();
    // corners.block(top_right, 0, 1, 3) = p2.transpose();
    // corners.block(top_left, 0, 1, 3) = p3.transpose();
  }

  // Form object points for SolvePnP
  std::vector<cv::Point3f> object_points;
  for (long i = 0; i < corners.rows(); i++) {
    Vec3 corner = corners.row(i);
    if (i == 0 || corner.isApprox(Vec3::Zero(3, 1)) == false) {
      object_points.emplace_back(corner(0), corner(1), corner(2));
    }
  }

  return object_points;
}

std::vector<cv::Point2f> AprilGrid::formImagePoints(
    const std::map<int, std::vector<cv::Point2f>> &tags) {
  std::vector<cv::Point2f> image_points;
  for (auto &tag : tags) {
    for (auto &corner : tag.second) {
      image_points.push_back(corner);
    }
  }

  return image_points;
}

int AprilGrid::solvePnP(const std::map<int, std::vector<cv::Point2f>> &tags,
                        const cv::Mat &K,
                        MatX &grid_points) {
  // SolvePnP
  const std::vector<cv::Point3f> object_points = this->formObjectPoints(tags);
  const std::vector<cv::Point2f> image_points = this->formImagePoints(tags);

  std::vector<cv::Point2f> image_points_ud;
  cv::Mat D(4, 1, cv::DataType<double>::type);
  D.at<double>(0) = 0.0678890809361792;
  D.at<double>(1) = 0.007709008000713503;
  D.at<double>(2) = -0.010307689056822753;
  D.at<double>(3) = 0.003975705353253985;
  cv::fisheye::undistortPoints(image_points, image_points_ud, K, D);
  for (size_t i = 0; i < image_points_ud.size(); i++) {
    cv::Point2f &p = image_points_ud[i];
    p.x = K.at<double>(0, 0) * p.x + K.at<double>(0, 2);
    p.y = K.at<double>(1, 1) * p.y + K.at<double>(1, 2);
  }

  cv::Mat D_empty;
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::solvePnP(object_points, image_points_ud, K, D_empty, rvec, tvec);

  // Convert Rodrigues rvec to rotation matrix
  cv::Mat R(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rvec, R);

  // Form transform
  // clang-format off
  Mat4 T_c_t;
  T_c_t << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at<double>(0),
           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at<double>(1),
           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at<double>(2),
           0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // std::cout << T_c_t << std::endl;
  // for (int i = 0; i < this->grid_points.rows(); i++) {
  //   Vec3 pt = this->grid_points.row(i).transpose();
  //   std::cout << "tag id: " << i << std::endl;
  //   std::cout << "point: " << pt.transpose() << std::endl;
  //   std::cout << "transformed: " << (T_c_t * pt.homogeneous()).transpose()
  //             << std::endl;
  //   std::cout << std::endl;
  // }

  grid_points.resize(object_points.size(), 3);
  int i = 0;
  for (auto &p : object_points) {
    const Vec3 pt{p.x, p.y, p.z};
    const Vec3 pt_transformed = (T_c_t * pt.homogeneous()).head(3);
    grid_points.block(i, 0, 1, 3) = pt_transformed.transpose();
    i++;
  }

  return 0;
}

} // namespace gvio
