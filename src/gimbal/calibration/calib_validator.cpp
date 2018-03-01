#include "gvio/gimbal/calibration/calib_validator.hpp"

namespace gvio {

CalibValidator::CalibValidator() {}

CalibValidator::~CalibValidator() {}

int CalibValidator::load(const std::string &calib_file) {
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

  return 0;
}

cv::Mat CalibValidator::validate(const cv::Mat &image) {
  cv::Mat result;

  return result;
}

} // namespace gvio
