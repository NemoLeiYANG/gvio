#include "gvio/gimbal/calibration/calib_validator.hpp"

namespace gvio {

CalibValidator::CalibValidator() {}

CalibValidator::~CalibValidator() {}

Mat3 CalibValidator::K() {
  Mat3 K;

  // First row
  K(0, 0) = this->intrinsics[0];
  K(0, 1) = 0.0;
  K(0, 2) = this->intrinsics[1];

  // Second row
  K(1, 0) = 0.0;
  K(1, 1) = this->intrinsics[2];
  K(1, 2) = this->intrinsics[3];

  // Thrid row
  K(2, 0) = 0.0;
  K(2, 1) = 0.0;
  K(2, 2) = 1.0;

  return K;
}

VecX CalibValidator::D() {
  VecX D;
  D.resize(4);

  // clang-format off
  D << this->distortion_coeffs(0),
       this->distortion_coeffs(1),
       this->distortion_coeffs(2),
       this->distortion_coeffs(3);
  // clang-format on

  return D;
}

int CalibValidator::load(const std::string &calib_file,
                         const std::string &target_file) {
  // Parse calib file
  ConfigParser parser;
  parser.addParam("camera_model", &this->camera_model);
  parser.addParam("distortion_model", &this->distortion_model);
  parser.addParam("distortion_coeffs", &this->distortion_coeffs);
  parser.addParam("intrinsics", &this->intrinsics);
  parser.addParam("resolution", &this->resolution);
  if (parser.load(calib_file) != 0) {
    LOG_ERROR("Failed to load calib file [%s]!", calib_file.c_str());
    return -1;
  }

  // Chessboard
  if (this->chessboard.load(target_file) != 0) {
    LOG_ERROR("Failed to load chessboard config!");
    return -1;
  }

  return 0;
}

cv::Mat CalibValidator::validate(const cv::Mat &image) {
  // Undistort image
  cv::Mat Knew;
  cv::Mat image_ud =
      pinhole_equi_undistort_image(this->K(), this->D(), image, Knew);

  // Find chessboard corners (with undistorted image)
  std::vector<cv::Point2f> corners;
  if (this->chessboard.detect(image_ud, corners) != 0) {
    return image;
  }

  // Solve PnP (with undistorted image)
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::solvePnP(chessboard.object_points,
               corners,
               Knew,
               cv::Mat::zeros(4, 1, CV_64FC1),
               rvec,
               tvec,
               false,
               cv::SOLVEPNP_ITERATIVE);

  // Project 3D point to image plane
  std::vector<cv::Point2f> image_points;
  cv::Mat D = convert(this->D());
  cv::fisheye::projectPoints(chessboard.object_points, // Object poitns
                             image_points,             // Image poitns
                             rvec,                     // Rodrigues vector
                             tvec,                     // Translation vector
                             Knew,                     // Camera intrinsics K
                             D);                       // Distortion vector D

  // Draw projected points
  for (size_t i = 0; i < image_points.size(); i++) {
    cv::circle(image_ud,              // Target image
               image_points[i],       // Center
               3.0,                   // Radius
               cv::Scalar(0, 0, 255), // Colour
               CV_FILLED,             // Thickness
               8);                    // Line type
  }

  return image_ud;
}

} // namespace gvio
