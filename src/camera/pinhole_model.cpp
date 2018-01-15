#include "gvio/camera/pinhole_model.hpp"

namespace gvio {

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

double PinholeModel::focalLengthX(const int image_width, const double fov) {
  double fx = ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
  return fx;
}

double PinholeModel::focalLengthY(const int image_height, const double fov) {
  double fy = ((image_height / 2.0) / tan(deg2rad(fov) / 2.0));
  return fy;
}

Vec2 PinholeModel::focalLength(const int image_width,
                               const int image_height,
                               const double fov) {
  auto focal_length = Vec2{PinholeModel::focalLengthX(image_width, fov),
                           PinholeModel::focalLengthY(image_height, fov)};
  return focal_length;
}

Mat34 PinholeModel::P(const Mat3 &R, const Vec3 &t) {
  Mat34 A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;

  Mat34 P;
  P = this->K * A;

  return P;
}

Vec2 PinholeModel::project(const Vec3 &X, const Mat3 &R, const Vec3 &t) {
  // Convert 3D features to homogenous coordinates
  const Vec4 X_homo = homogeneous(X);

  // Project 3D point to image plane
  const Mat34 P = this->P(R, t);
  Vec3 x = P * X_homo;

  // Normalize pixel coordinates
  x(0) = x(0) / x(2);
  x(1) = x(1) / x(2);
  x(2) = x(2) / x(2);

  return x.block(0, 0, 2, 1);
}

Vec2 PinholeModel::pixel2image(const Vec2 &pixel) {
  Vec2 pt((pixel(0) - this->cx) / this->fx, (pixel(1) - this->cy) / this->fy);
  return pt;
}

Vec2 PinholeModel::pixel2image(const cv::Point2f &pixel) {
  return this->pixel2image(Vec2{pixel.x, pixel.y});
}

Vec2 PinholeModel::pixel2image(const cv::KeyPoint &kp) {
  return this->pixel2image(Vec2{kp.pt.x, kp.pt.y});
}

Vec2 PinholeModel::pixel2image(const Vec2 &pixel) const {
  Vec2 pt((pixel(0) - this->cx) / this->fx, (pixel(1) - this->cy) / this->fy);
  return pt;
}

Vec2 PinholeModel::pixel2image(const cv::Point2f &pixel) const {
  return this->pixel2image(Vec2{pixel.x, pixel.y});
}

Vec2 PinholeModel::pixel2image(const cv::KeyPoint &kp) const {
  return this->pixel2image(Vec2{kp.pt.x, kp.pt.y});
}

MatX PinholeModel::observedFeatures(const MatX &features,
                                    const Vec3 &rpy,
                                    const Vec3 &t,
                                    std::vector<int> &mask) {
  // Rotation matrix
  Mat3 R = euler123ToRot(rpy);

  // projection matrix
  Mat34 P = this->P(R, t);

  // Check which features are observable from camera
  std::vector<Vec2> observed;
  for (int i = 0; i < features.cols(); i++) {
    // Project 3D world point to 2D image plane
    const Vec3 point = features.col(i);
    Vec3 img_pt = P * homogeneous(point);

    // Check to see if feature is valid and infront of camera
    if (img_pt(2) < 1.0) {
      continue; // skip this feature! It is not infront of camera
    }

    // Normalize pixels
    img_pt(0) = img_pt(0) / img_pt(2);
    img_pt(1) = img_pt(1) / img_pt(2);
    img_pt(2) = img_pt(2) / img_pt(2);

    // Check to see if feature observed is within image plane
    const bool x_ok = (img_pt(0) < this->image_width) && (img_pt(0) > 0.0);
    const bool y_ok = (img_pt(1) < this->image_height) && (img_pt(1) > 0.0);
    if (x_ok && y_ok) {
      observed.emplace_back(img_pt(0), img_pt(1));
      mask.push_back(i);
    }
  }

  // Convert vector of Vec3 to MatX
  MatX result;
  result.resize(2, observed.size());

  int index = 0;
  for (auto v : observed) {
    result.block(0, index, 2, 1) = v;
    index++;
  }

  return result;
}

} // namespace gvio
