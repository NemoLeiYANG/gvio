#include "gvio/calibration/camchain.hpp"

namespace gvio {

CameraProperty::CameraProperty() {}

CameraProperty::CameraProperty(const int camera_index,
                               const double fx,
                               const double fy,
                               const double cx,
                               const double cy,
                               const int image_width,
                               const int image_height)
    : camera_index{camera_index}, camera_model{"pinhole"}, distortion_model{},
      resolution{image_width, image_height} {
  intrinsics = Vec4{fx, fy, cx, cy};
}

Mat3 CameraProperty::K() {
  const double fx = this->intrinsics(0);
  const double fy = this->intrinsics(1);
  const double cx = this->intrinsics(2);
  const double cy = this->intrinsics(3);

  // clang-format off
  Mat3 K;
  K << fx, 0.0, cx,
       0.0, fy, cy,
       0.0, 0.0, 1.0;
  // clang-format on

  return K;
}

VecX CameraProperty::D() {
  if (distortion_model == "equidistant") {
    const double k1 = this->distortion_coeffs(0);
    const double k2 = this->distortion_coeffs(1);
    const double k3 = this->distortion_coeffs(2);
    const double k4 = this->distortion_coeffs(3);
    VecX D = zeros(4, 1);
    D << k1, k2, k3, k4;
    return D;

  } else if (distortion_model == "radtan") {
    const double k1 = this->distortion_coeffs(0);
    const double k2 = this->distortion_coeffs(1);
    const double p1 = this->distortion_coeffs(2);
    const double p2 = this->distortion_coeffs(3);
    const double k3 = this->distortion_coeffs(4);
    VecX D = zeros(5, 1);
    D << k1, k2, p1, p2, k3;
    return D;
  } else {
    LOG_ERROR("Unsupported distortion model [%s]!", distortion_model.c_str());
    VecX D;
    return D;
  }
}

int CameraProperty::undistortPoints(
    const std::vector<cv::Point2f> &image_points,
    std::vector<cv::Point2f> &image_points_ud) {

  if (distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(image_points,
                                 image_points_ud,
                                 convert(this->K()),
                                 convert(this->D()));

  } else if (distortion_model == "radtan") {
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

int CameraProperty::undistortPoint(const cv::Point2f &image_point,
                                   cv::Point2f &image_point_ud) {
  std::vector<cv::Point2f> points = {image_point};
  std::vector<cv::Point2f> points_ud = {image_point};

  if (distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(points,
                                 points_ud,
                                 convert(this->K()),
                                 convert(this->D()));

  } else if (distortion_model == "radtan") {
    cv::undistortPoints(points,
                        points_ud,
                        convert(this->K()),
                        convert(this->D()));

  } else {
    LOG_ERROR("Unsupported distortion model [%s]!", distortion_model.c_str());
    exit(0);
    return -1;
  }

  // Set result
  image_point_ud = points_ud[0];

  return 0;
}

int CameraProperty::undistortImage(const cv::Mat &image,
                                   const double balance,
                                   cv::Mat &image_ud,
                                   cv::Mat &K_ud) {
  const cv::Mat K = convert(this->K());
  const cv::Mat D = convert(this->D());

  if (distortion_model == "equidistant") {
    // Estimate new camera matrix first
    const cv::Size img_size = {image.cols, image.rows};
    const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K,
                                                            D,
                                                            img_size,
                                                            R,
                                                            K_ud,
                                                            balance);

    // Undistort image
    cv::fisheye::undistortImage(image, image_ud, K, D, K_ud);

  } else if (distortion_model == "radtan") {
    // Undistort image
    K_ud = K.clone();
    cv::undistort(image, image_ud, K, D, K_ud);

  } else {
    LOG_ERROR("Unsupported distortion model [%s]!", distortion_model.c_str());
    return -1;
  }

  return 0;
}

int CameraProperty::undistortImage(const cv::Mat &image,
                                   const double balance,
                                   cv::Mat &image_ud) {
  cv::Mat K_ud;
  return this->undistortImage(image, balance, image_ud, K_ud);
}

int CameraProperty::project(const MatX &X, MatX &pixels) {
  pixels.resize(2, X.cols());

  if (this->distortion_model == "equidistant") {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      const Vec2 pixel = project_pinhole_equi(this->K(), this->D(), p);
      pixels.col(i) = pixel;
    }

  } else if (this->distortion_model == "radtan") {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      const Vec2 pixel = project_pinhole_radtan(this->K(), this->D(), p);
      pixels.col(i) = pixel;
    }

  } else if (this->distortion_model == "" && this->camera_model == "pinhole") {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      pixels.col(i) = pinhole_project(this->K(), p);
    }

  } else {
    LOG_ERROR("Distortion model and camera model are not set!");
    return -1;
  }

  return 0;
}

int CameraProperty::project(const Vec3 &X, Vec2 &pixel) {
  if (this->distortion_model == "equidistant") {
    pixel = project_pinhole_equi(this->K(), this->D(), X);

  } else if (this->distortion_model == "radtan") {
    pixel = project_pinhole_radtan(this->K(), this->D(), X);

  } else if (this->distortion_model == "" && this->camera_model == "pinhole") {
    pixel = pinhole_project(this->K(), X);
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

} // namespace gvio
