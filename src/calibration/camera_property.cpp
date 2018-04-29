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
  this->intrinsics = Vec4{fx, fy, cx, cy};
}

CameraProperty::CameraProperty(const int camera_index,
                               const Mat3 &K,
                               const Vec2 &resolution)
    : camera_index{camera_index}, camera_model{"pinhole"}, distortion_model{},
      resolution{resolution} {
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  this->intrinsics = Vec4{fx, fy, cx, cy};
}

CameraProperty::CameraProperty(const int camera_index,
                               const std::string &camera_model,
                               const Mat3 &K,
                               const std::string &distortion_model,
                               const VecX &D,
                               const Vec2 &resolution)
    : camera_index{camera_index}, camera_model{camera_model},
      distortion_model{distortion_model}, distortion_coeffs{D},
      resolution{resolution} {
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  this->intrinsics = Vec4{fx, fy, cx, cy};

  // Some camera calibration results do not provide radtan k3, in the following
  // we set radtan k3 = 0.0 if the provided distortion vector is of size 4. We
  // assume:
  //
  //   D (size 4) = k1, k2, p1, p2
  //   D (size 5) = k1, k2, p1, p2, k3
  //
  if (distortion_model == "radtan" && D.size() == 4) {
    this->distortion_coeffs = zeros(5, 1);
    this->distortion_coeffs << D, 0.0;
  }
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

  } else if (distortion_model.empty()) {
    FATAL("Distortion model not set!");

  } else {
    FATAL("Unsupported distortion model [%s]!", distortion_model.c_str());
  }
}

std::vector<cv::Point2f> CameraProperty::undistortPoints(
    const std::vector<cv::Point2f> &image_points, const Mat3 &rect_mat) {
  std::vector<cv::Point2f> image_points_ud;

  const cv::Matx33d K_new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

  if (distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(image_points,
                                 image_points_ud,
                                 convert(this->K()),
                                 convert(this->D()),
                                 convert(rect_mat),
                                 K_new);

  } else if (distortion_model == "radtan") {
    cv::undistortPoints(image_points,
                        image_points_ud,
                        convert(this->K()),
                        convert(this->D()),
                        convert(rect_mat),
                        K_new);

  } else if (distortion_model.empty()) {
    FATAL("Distortion model not set!");

  } else {
    FATAL("Unsupported distortion model [%s]!", distortion_model.c_str());
  }

  return image_points_ud;
}

cv::Point2f CameraProperty::undistortPoint(const cv::Point2f &image_point,
                                           const Mat3 &rect_mat) {
  std::vector<cv::Point2f> points = {image_point};
  std::vector<cv::Point2f> points_ud = this->undistortPoints(points, rect_mat);
  return points_ud[0];
}

std::vector<cv::Point2f>
CameraProperty::distortPoints(const std::vector<cv::Point2f> &points) {
  std::vector<cv::Point2f> points_distorted;

  if (distortion_model == "radtan") {
    std::vector<cv::Point3f> points_homo;
    cv::convertPointsToHomogeneous(points, points_homo);

    cv::projectPoints(points_homo,
                      cv::Vec3d::zeros(), // rvec
                      cv::Vec3d::zeros(), // tvec
                      convert(this->K()), // Intrinsics matrix K
                      convert(this->D()), // Distortion coefficients D
                      points_distorted);

  } else if (distortion_model == "equidistant") {
    cv::fisheye::distortPoints(points,
                               points_distorted,
                               convert(this->K()),
                               convert(this->D()));

  } else if (distortion_model.empty()) {
    FATAL("Distortion model not set!");

  } else {
    FATAL("Unsupported distortion model [%s]!", distortion_model.c_str());
  }

  return points_distorted;
}

cv::Mat CameraProperty::undistortImage(const cv::Mat &image,
                                       const double balance,
                                       cv::Mat &K_ud) {
  const cv::Mat K = convert(this->K());
  const cv::Mat D = convert(this->D());
  cv::Mat image_ud;

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

  } else if (distortion_model.empty()) {
    FATAL("Distortion model not set!");

  } else {
    FATAL("Unsupported distortion model [%s]!", distortion_model.c_str());
  }

  return image_ud;
}

cv::Mat CameraProperty::undistortImage(const cv::Mat &image,
                                       const double balance) {
  cv::Mat K_ud;
  return this->undistortImage(image, balance, K_ud);
}

MatX CameraProperty::project(const MatX &X) {
  MatX pixels;
  pixels.resize(2, X.cols());

  if (this->camera_model == "pinhole" &&
      this->distortion_model == "equidistant") {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      const Vec2 pixel = project_pinhole_equi(this->K(), this->D(), p);
      pixels.col(i) = pixel;
    }

  } else if (this->camera_model == "pinhole" &&
             this->distortion_model == "radtan") {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      const Vec2 pixel = project_pinhole_radtan(this->K(), this->D(), p);
      pixels.col(i) = pixel;
    }

  } else if (this->camera_model == "pinhole" &&
             this->distortion_model.empty()) {
    for (long i = 0; i < X.cols(); i++) {
      const Vec3 p = X.col(i);
      pixels.col(i) = pinhole_project(this->K(), p);
    }

  } else {
    FATAL("Distortion model and camera model are not set!");
  }

  return pixels;
}

Vec2 CameraProperty::project(const Vec3 &X) {
  Vec2 pixel;

  if (this->camera_model == "pinhole" &&
      this->distortion_model == "equidistant") {
    pixel = project_pinhole_equi(this->K(), this->D(), X);

  } else if (this->camera_model == "pinhole" &&
             this->distortion_model == "radtan") {
    pixel = project_pinhole_radtan(this->K(), this->D(), X);

  } else if (this->camera_model == "pinhole" &&
             this->distortion_model.empty()) {
    pixel = pinhole_project(this->K(), X);
  }

  return pixel;
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
