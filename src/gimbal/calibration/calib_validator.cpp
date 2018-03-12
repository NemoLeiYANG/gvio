#include "gvio/gimbal/calibration/calib_validator.hpp"

namespace gvio {

std::ostream &operator<<(std::ostream &os, const CameraProperty &cam) {
  os << "camera_model: " << cam.camera_model << std::endl;
  os << "distortion_model: " << cam.distortion_model << std::endl;
  os << "distortion_coeffs: " << cam.distortion_coeffs.transpose() << std::endl;
  os << "intrinsics: " << cam.intrinsics.transpose() << std::endl;
  os << "resolution: " << cam.resolution.transpose() << std::endl;
  return os;
}

CalibValidator::CalibValidator() {}

CalibValidator::~CalibValidator() {}

Mat3 CalibValidator::K(const int cam_id) {
  assert(cam_id >= 0);
  assert(cam_id < this->cam.size());

  Mat3 K;
  // First row
  K(0, 0) = this->cam[cam_id].intrinsics[0];
  K(0, 1) = 0.0;
  K(0, 2) = this->cam[cam_id].intrinsics[1];

  // Second row
  K(1, 0) = 0.0;
  K(1, 1) = this->cam[cam_id].intrinsics[2];
  K(1, 2) = this->cam[cam_id].intrinsics[3];

  // Thrid row
  K(2, 0) = 0.0;
  K(2, 1) = 0.0;
  K(2, 2) = 1.0;

  return K;
}

VecX CalibValidator::D(const int cam_id) {
  assert(cam_id >= 0);
  assert(cam_id < this->cam.size());

  VecX D;
  D.resize(4);

  // clang-format off
  D << this->cam[cam_id].distortion_coeffs(0),
       this->cam[cam_id].distortion_coeffs(1),
       this->cam[cam_id].distortion_coeffs(2),
       this->cam[cam_id].distortion_coeffs(3);
  // clang-format on

  return D;
}

int CalibValidator::load(const int nb_cameras,
                         const std::string &calib_file,
                         const std::string &target_file) {
  assert(nb_cameras > 0);
  assert(calib_file.empty() == false);
  assert(target_file.empty() == false);

  // Parse calib file
  ConfigParser parser;
  CameraProperty cam0, cam1, cam2;
  for (int i = 0; i < nb_cameras; i++) {
    switch (i) {
      case 0:
        parser.addParam("cam0.camera_model", &cam0.camera_model);
        parser.addParam("cam0.distortion_model", &cam0.distortion_model);
        parser.addParam("cam0.distortion_coeffs", &cam0.distortion_coeffs);
        parser.addParam("cam0.intrinsics", &cam0.intrinsics);
        parser.addParam("cam0.resolution", &cam0.resolution);
        break;
      case 1:
        parser.addParam("cam1.camera_model", &cam1.camera_model);
        parser.addParam("cam1.distortion_model", &cam1.distortion_model);
        parser.addParam("cam1.distortion_coeffs", &cam1.distortion_coeffs);
        parser.addParam("cam1.intrinsics", &cam1.intrinsics);
        parser.addParam("cam1.resolution", &cam1.resolution);
        break;
      case 2:
        parser.addParam("cam2.camera_model", &cam2.camera_model);
        parser.addParam("cam2.distortion_model", &cam2.distortion_model);
        parser.addParam("cam2.distortion_coeffs", &cam2.distortion_coeffs);
        parser.addParam("cam2.intrinsics", &cam2.intrinsics);
        parser.addParam("cam2.resolution", &cam2.resolution);
        break;
    }
  }
  if (parser.load(calib_file) != 0) {
    LOG_ERROR("Failed to load calib file [%s]!", calib_file.c_str());
    return -1;
  }
  this->cam = {cam0, cam1, cam2};

  // Chessboard
  if (this->chessboard.load(target_file) != 0) {
    LOG_ERROR("Failed to load chessboard config!");
    return -1;
  }

  return 0;
}

cv::Mat CalibValidator::validate(const int cam_id, const cv::Mat &image) {
  assert(cam_id >= 0);
  assert(cam_id < this->cam.size());
  assert(image.empty() == false);

  // Undistort image
  cv::Mat Knew;
  cv::Mat image_ud = pinhole_equi_undistort_image(this->K(cam_id),
                                                  this->D(cam_id),
                                                  image,
                                                  Knew);

  // Find chessboard corners (with undistorted image)
  std::vector<cv::Point2f> corners;
  if (this->chessboard.detect(image_ud, corners) != 0) {
    return image_ud;
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
  cv::Mat D = convert(this->D(cam_id));
  cv::fisheye::projectPoints(chessboard.object_points, // Object points
                             image_points,             // Image points
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
