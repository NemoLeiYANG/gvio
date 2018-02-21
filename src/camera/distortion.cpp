#include "gvio/camera/distortion.hpp"

namespace gvio {

MatX radtan_distort(const double k1,
                    const double k2,
                    const double k3,
                    const double p1,
                    const double p2,
                    const MatX points) {
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
                  const MatX points) {
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

  // Project rad-tan distorted points to image plane
  const int nb_points = points.rows();
  MatX distorted_points{nb_points, 2};
  distorted_points.col(0) = x_dash;
  distorted_points.col(1) = y_dash;
  return distorted_points;
}

} // namespace gvio
