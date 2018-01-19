#include "gvio/sim/camera.hpp"

namespace gvio {

MatX VirtualCamera::observedFeatures(const MatX &features,
                                     const Vec3 &rpy_G,
                                     const Vec3 &t_G,
                                     std::vector<int> &mask) {
  // Rotation matrix
  // Convert from Global frame NWU to Camera frame EDN
  // NWU: (x - forward, y - left, z - up)
  // EDN: (x - right, y - down, z - forward)
  const Vec3 rpy_C{-rpy_G(1), -rpy_G(2), rpy_G(0)};
  const Mat3 R = euler123ToRot(rpy_C);

  // Check which features are observable from camera
  std::vector<Vec2> observed;
  for (int i = 0; i < features.rows(); i++) {
    // Transform point from global frame to camera frame
    const Vec3 pt_G = features.row(i).transpose();
    const Vec3 pt_C = rotx(-M_PI / 2.0) * rotz(-M_PI / 2.0) * pt_G;

    // Transform translation from global frame to camera frame
    const Vec3 t_C = rotx(-M_PI / 2.0) * rotz(-M_PI / 2.0) * t_G;

    // Project 3D world point to 2D image plane
    Vec3 img_pt = this->camera_model.project(homogeneous(pt_C), R, t_C);

    // Check to see if feature is valid and infront of camera
    if (img_pt(2) < 1.0) {
      continue; // skip this feature! It is not infront of camera
    }

    // Normalize pixels
    img_pt(0) = img_pt(0) / img_pt(2);
    img_pt(1) = img_pt(1) / img_pt(2);
    img_pt(2) = img_pt(2) / img_pt(2);

    // Check to see if feature observed is within image plane
    const int image_width = this->camera_model.image_width;
    const int image_height = this->camera_model.image_height;
    const bool x_ok = (img_pt(0) < image_width) && (img_pt(0) > 0.0);
    const bool y_ok = (img_pt(1) < image_height) && (img_pt(1) > 0.0);
    if (x_ok && y_ok) {
      observed.emplace_back(img_pt(0), img_pt(1));
      mask.push_back(i);
    }
  }

  // Convert vector of Vec2 to MatX
  MatX result = zeros(observed.size(), 2);
  for (size_t i = 0; i < observed.size(); i++) {
    result.block(i, 0, 1, 2) = observed[i].transpose();
  }

  return result;
}

} // namespace gvio
