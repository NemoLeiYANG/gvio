#include "gvio/gimbal/calibration/calib_validator.hpp"

namespace gvio {

CalibValidator::CalibValidator() {}

CalibValidator::~CalibValidator() {}

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
  cv::Mat result = image.clone();

  // Find corners
  std::vector<cv::Point2f> corners;
  if (this->chessboard.detect(image, corners) != 0) {
    return result;
  }

  // Form camera matrix K
  cv::Mat K(3, 3, cv::DataType<double>::type);
  K.at<double>(0, 0) = this->intrinsics[0];
  K.at<double>(0, 1) = 0.0;
  K.at<double>(0, 2) = this->intrinsics[1];
  K.at<double>(1, 0) = 0.0;
  K.at<double>(1, 1) = this->intrinsics[2];
  K.at<double>(1, 2) = this->intrinsics[3];
  K.at<double>(2, 0) = 0.0;
  K.at<double>(2, 1) = 0.0;
  K.at<double>(2, 2) = 1.0;

  // Calculate corner positions
  MatX X;
  int retval = this->chessboard.calcCornerPositions(corners, K, X);
  if (retval != 0) {
    LOG_ERROR("Failed to calculate corner positions!");
    return result;
  }

  // Project 3D points to image
  this->chessboard.project3DPoints(X, K, result);

  return result;
}

} // namespace gvio
