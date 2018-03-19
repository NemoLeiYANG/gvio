#include "gvio/gimbal/calibration/camchain.hpp"

namespace gvio {

Mat3 CameraProperty::K() {
  const double fx = this->intrinsics(0);
  const double fy = this->intrinsics(1);
  const double cx = this->intrinsics(2);
  const double cy = this->intrinsics(3);

  Mat3 K;
  // clang-format off
  K << fx, 0.0, cx,
       0.0, fy, cy,
       0.0, 0.0, 1.0;
  // clang-format on

  return K;
}

VecX CameraProperty::D() { return this->distortion_coeffs; }

int CameraProperty::undistortPoints(
    const std::vector<cv::Point2f> &image_points,
    std::vector<cv::Point2f> &image_points_ud) {
  std::vector<cv::Point2f> points_ud;

  if (distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(image_points,
                                 image_points_ud,
                                 convert(this->K()),
                                 convert(this->D()));

  } else if (distortion_model == "radial-tangential") {
    cv::undistortPoints(image_points,
                        image_points_ud,
                        convert(this->K()),
                        convert(this->D()));

  } else {
    LOG_ERROR("Unsupported distortion model [%s]!", distortion_model.c_str());
    return -1;
  }

  return 0;
}

int CameraProperty::project(const MatX &X, MatX &pixels) {
  pixels.resize(2, X.cols());

  if (this->distortion_model == "equidistant") {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      const Vec2 pixel = project_pinhole_equi(this->K(), this->D(), p);
      pixels.col(i) = pixel;
    }

  } else if (distortion_model == "radial-tangential") {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      const Vec2 pixel = project_pinhole_radtan(this->K(), this->D(), p);
      pixels.col(i) = pixel;
    }

  } else {
    LOG_ERROR("Unsupported distortion model [%s]!", distortion_model.c_str());
    return -1;
  }

  return 0;
}

std::ostream &operator<<(std::ostream &os, const CameraProperty &cam) {
  os << "camera_model: " << cam.camera_model << std::endl;
  os << "distortion_model: " << cam.distortion_model << std::endl;
  os << "distortion_coeffs: " << cam.distortion_coeffs.transpose() << std::endl;
  os << "intrinsics: " << cam.intrinsics.transpose() << std::endl;
  os << "resolution: " << cam.resolution.transpose() << std::endl;
  return os;
}

Camchain::Camchain() {}

Camchain::~Camchain() {}

int Camchain::load(const int nb_cameras, const std::string &camchain_file) {
  assert(nb_cameras > 0);
  assert(camchain_file.empty() == false);

  // Parse camchain file
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
  parser.addParam("T_C1_C0", &this->T_C1_C0);
  parser.addParam("T_C2_C0.tau_s", &this->tau_s);
  parser.addParam("T_C2_C0.tau_d", &this->tau_d);
  parser.addParam("T_C2_C0.w1", &this->w1);
  parser.addParam("T_C2_C0.w2", &this->w2);
  parser.addParam("T_C2_C0.theta1_offset", &this->theta1_offset);
  parser.addParam("T_C2_C0.theta2_offset", &this->theta2_offset);
  if (parser.load(camchain_file) != 0) {
    return -1;
  }
  this->cam = {cam0, cam1, cam2};

  return 0;
}

} // namespace gvio
