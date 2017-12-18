#include "gvio/camera/pinhole_model.hpp"

namespace gvio {

double PinHoleModel::focalLengthX(const int image_width, const double fov) {
  double fx = ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
  return fx;
}

double PinHoleModel::focalLengthY(const int image_height, const double fov) {
  double fy = ((image_height / 2.0) / tan(deg2rad(fov) / 2.0));
  return fy;
}

Vec2 PinHoleModel::focalLength(const int image_width,
                               const int image_height,
                               const double fov) {
  auto focal_length = Vec2{PinHoleModel::focalLengthX(image_width, fov),
                           PinHoleModel::focalLengthY(image_height, fov)};
  return focal_length;
}

Mat34 PinHoleModel::P(const Mat3 &R, const Vec3 &t) {
  Mat34 A;
  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 1) = -R * t;

  Mat34 P;
  P = this->K * A;

  return P;
}

Vec3 PinHoleModel::project(const Vec3 &X, const Mat3 &R, const Vec3 &t) {
  // Convert 3D features to homogenous coordinates
  const Vec4 X_homo = homogeneous(X);

  // Project 3D point to image plane
  const Mat34 P = this->P(R, t);
  Vec3 x = P * X_homo;

  // Normalize pixel coordinates
  x(0) = x(0) / x(2);
  x(1) = x(1) / x(2);
  x(2) = x(2) / x(2);

  return x;
}

Vec2 PinHoleModel::pixel2image(const Vec2 &pixel) {
  Vec2 pt((pixel(0) - this->cx) / this->fx, (pixel(1) - this->cy) / this->fy);
  return pt;
}

// TODO: Need to a more robust test for when feature is behind camera
MatX PinHoleModel::observedFeatures(const MatX &features,
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
