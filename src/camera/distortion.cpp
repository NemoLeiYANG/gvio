#include "gvio/camera/distortion.hpp"

namespace gvio {

MatX radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const MatX &points) {
  // Asserts
  assert(points.cols() == 3);

  // Setup
  const Eigen::ArrayXd z = points.col(2).array();
  const Eigen::ArrayXd x = points.col(0).array() / z.array();
  const Eigen::ArrayXd y = points.col(1).array() / z.array();

  // Apply radial distortion factor
  // clang-format off
  const int nb_points = points.rows();
  const Eigen::ArrayXd r2 = x.pow(2) + y.pow(2);
  const Eigen::ArrayXd one = ones(nb_points, 1).array();
  const Eigen::ArrayXd x_dash = x * (one + (k1 * r2) + (k2 * r2.pow(2)) + (k3 * r2.pow(3)));
  const Eigen::ArrayXd y_dash = y * (one + (k1 * r2) + (k2 * r2.pow(2)) + (k3 * r2.pow(3)));
  // clang-format on

  // Apply tangential distortion factor
  // clang-format off
  const Eigen::ArrayXd x_ddash = x_dash + (2 * p1 * x * y + p2 * (r2 + 2 * x.pow(2)));
  const Eigen::ArrayXd y_ddash = x_dash + (p1 * (r2 + 2 * y.pow(2)) + 2 * p2 * x * y);
  // clang-format on

  // Project rad-tan distorted points to image plane
  MatX distorted_points{nb_points, 2};
  distorted_points.col(0) = x_ddash;
  distorted_points.col(1) = y_ddash;

  return distorted_points;
}

MatX equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const MatX &points) {
  // Asserts
  assert(points.cols() == 3);

  // Setup
  const Eigen::ArrayXd z = points.col(2).array();
  const Eigen::ArrayXd x = points.col(0).array() / z.array();
  const Eigen::ArrayXd y = points.col(1).array() / z.array();
  const Eigen::ArrayXd r = (x.pow(2) + y.pow(2)).sqrt();

  // Apply equi distortion
  // clang-format off
  const Eigen::ArrayXd theta = r.atan();
  const Eigen::ArrayXd theta_d = theta + k1 * theta.pow(3) + k2 * theta.pow(5) + k3 * theta.pow(7) + k4 * theta.pow(9);
  const Eigen::ArrayXd x_dash = (theta_d / r) * x;
  const Eigen::ArrayXd y_dash = (theta_d / r) * y;
  // clang-format on

  // Project equi distorted points to image plane
  const int nb_points = points.rows();
  MatX distorted_points{nb_points, 2};
  distorted_points.col(0) = x_dash;
  distorted_points.col(1) = y_dash;
  return distorted_points;
}

Vec2 equi_distort(const double k1,
                  const double k2,
                  const double k3,
                  const double k4,
                  const Vec3 &point) {
  const double z = point(2);
  const double x = point(0) / z;
  const double y = point(1) / z;
  const double r = sqrt(pow(x, 2) + pow(y, 2));

  // Apply equi distortion
  // clang-format off
  const double theta = atan(r);
  const double th2 = pow(theta, 2);
  const double th4 = pow(theta, 4);
  const double th6 = pow(theta, 6);
  const double th8 = pow(theta, 8);
  const double theta_d = theta * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double x_dash = (theta_d / r) * x;
  const double y_dash = (theta_d / r) * y;
  // clang-format on

  // Project equi distorted point to image plane
  return Vec2{x_dash, y_dash};
}

void equi_undistort(const double k1,
                    const double k2,
                    const double k3,
                    const double k4,
                    Vec2 &p) {
  const double thetad = sqrt(p(0) * p(0) + p(1) * p(1));

  double theta = thetad; // Initial guess
  for (int i = 20; i > 0; i--) {
    const double th2 = theta * theta;
    const double th4 = th2 * th2;
    const double th6 = th4 * th2;
    const double th8 = th4 * th4;
    theta = thetad / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const double scaling = tan(theta) / thetad;
  p(0) *= scaling;
  p(1) *= scaling;
}

cv::Mat pinhole_equi_undistort_image(const Mat3 &K,
                                     const VecX &D,
                                     const cv::Mat &image,
                                     const double balance,
                                     cv::Mat &Knew) {
  // Estimate new camera matrix first
  const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(convert(K),
                                                          convert(D),
                                                          image.size(),
                                                          R,
                                                          Knew,
                                                          balance);

  // Undistort image
  cv::Mat image_ud;
  cv::fisheye::undistortImage(image, image_ud, convert(K), convert(D), Knew);

  return image_ud;
}

cv::Mat pinhole_equi_undistort_image(const Mat3 &K,
                                     const VecX &D,
                                     const cv::Mat &image,
                                     cv::Mat &Knew) {
  return pinhole_equi_undistort_image(K, D, image, 0.0, Knew);
}

} // namespace gvio
