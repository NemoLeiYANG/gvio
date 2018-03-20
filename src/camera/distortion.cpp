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

Vec2 radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const Vec3 &point) {
  // Setup
  const double x_dash = point(0) / point(2);
  const double y_dash = point(1) / point(2);

  // Radial distortion factor
  const double r2 = (x_dash * x_dash) + (y_dash * y_dash);
  const double r4 = r2 * r2;
  const double r6 = r2 * r4;
  const double temp = 1 + (k1 * r2) + (k2 * r4) + (k3 * r6);

  // Tangential distortion factor
  // clang-format off
  const double x_ddash = x_dash * temp + (2 * p1 * x_dash * y_dash + p2 * (r2 + 2 * (x_dash * x_dash)));
  const double y_ddash = y_dash * temp + (p1 * (r2 + 2 * (y_dash * y_dash)) + 2 * p2 * x_dash * y_dash);
  // clang-format on

  // Project rad-tan distorted point to image plane
  Vec2 distorted_point{x_ddash, y_ddash};
  return distorted_point;
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
  const double th2 = theta * theta;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
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

Vec2 project_pinhole_radtan(const Mat3 &K, const VecX &D, const Vec3 &X) {
  // Apply equi distortion
  const double k1 = D(0);
  const double k2 = D(1);
  const double p1 = D(2);
  const double p2 = D(3);
  const double k3 = D(4);
  const Vec2 x_distorted = radtan_distort(k1, k2, k3, p1, p2, X);

  // Project equi distorted point to image plane
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  const Vec2 pixel{fx * x_distorted(0) + cx, fy * x_distorted(1) + cy};

  return pixel;
}

Vec2 project_pinhole_equi(const Mat3 &K, const Vec4 &D, const Vec3 &X) {
  // Distort point
  const Vec2 x_distorted = equi_distort(D(0), D(1), D(2), D(3), X);

  // Project equi distorted point to image plane
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  const Vec2 pixel{fx * x_distorted(0) + cx, fy * x_distorted(1) + cy};

  return pixel;
}

} // namespace gvio
