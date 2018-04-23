#include "gvio/camera/pinhole_model.hpp"

namespace gvio {

Mat3 pinhole_K(const Vec4 &intrinsics) {
  Mat3 K;

  // clang-format off
  K << intrinsics(0), 0.0, intrinsics(2),
       0.0, intrinsics(1), intrinsics(3),
       0.0, 0.0, 1.0;
  // clang-format on

  return K;
}

double pinhole_focal_length(const int image_width, const double fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

Vec2 pinhole_focal_length(const Vec2 &image_size,
                          const double hfov,
                          const double vfov) {
  const double fx = ((image_size(0) / 2.0) / tan(deg2rad(hfov) / 2.0));
  const double fy = ((image_size(1) / 2.0) / tan(deg2rad(vfov) / 2.0));
  return Vec2{fx, fy};
}

Mat34 pinhole_projection_matrix(const Mat3 &K, const Mat3 &R, const Vec3 &t) {
  Mat34 A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;

  const Mat34 P = K * A;
  return P;
}

Vec2 pinhole_project(const Mat3 &K, const Vec3 &X) {
  const Vec3 x = K * X;
  return Vec2{x(0) / x(2), x(1) / x(2)};
}

Vec3 pinhole_project(const Mat3 &K,
                     const Mat3 &R,
                     const Vec3 &t,
                     const Vec4 &X) {
  Mat34 A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;

  // Form projection matrix
  const Mat34 P = K * A;
  const Vec3 x = P * X;
  return x;
}

Vec2 pinhole_project(const Mat3 &K,
                     const Mat3 &R,
                     const Vec3 &t,
                     const Vec3 &X) {
  const Vec4 X_homo = X.homogeneous();
  const Vec3 x = pinhole_project(K, R, t, X_homo);
  return Vec2{x(0) / x(2), x(1) / x(2)};
}

Vec2 pinhole_pixel2ideal(const double fx,
                         const double fy,
                         const double cx,
                         const double cy,
                         const Vec2 &pixel) {
  Vec2 pt((pixel(0) - cx) / fx, (pixel(1) - cy) / fy);
  return pt;
}

Vec2 pinhole_pixel2ideal(const Mat3 &K, const Vec2 &pixel) {
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  return pinhole_pixel2ideal(fx, fy, cx, cy, pixel);
}

int PinholeModel::configure(const std::string &config_file) {
  // Load config file
  ConfigParser parser;
  parser.addParam("image_width", &this->image_width);
  parser.addParam("image_height", &this->image_height);
  parser.addParam("fx", &this->fx);
  parser.addParam("fy", &this->fy);
  parser.addParam("cx", &this->cx);
  parser.addParam("cy", &this->cy);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Form the intrinsics matrix
  this->K = Mat3::Zero();
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;
  K(2, 2) = 1.0;

  return 0;
}

Mat34 PinholeModel::P(const Mat3 &R, const Vec3 &t) {
  return pinhole_projection_matrix(this->K, R, t);
}

Vec2 PinholeModel::project(const Vec3 &X, const Mat3 &R, const Vec3 &t) {
  return pinhole_project(this->K, R, t, X);
}

Vec3 PinholeModel::project(const Vec4 &X, const Mat3 &R, const Vec3 &t) {
  return pinhole_project(this->K, R, t, X);
}

Vec2 PinholeModel::pixel2ideal(const Vec2 &pixel) {
  return pinhole_pixel2ideal(this->K, pixel);
}

Vec2 PinholeModel::pixel2ideal(const cv::Point2f &pixel) {
  return this->pixel2ideal(Vec2{pixel.x, pixel.y});
}

Vec2 PinholeModel::pixel2ideal(const cv::KeyPoint &kp) {
  return this->pixel2ideal(Vec2{kp.pt.x, kp.pt.y});
}

} // namespace gvio
