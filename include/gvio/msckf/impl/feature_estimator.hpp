#ifndef GVIO_MSCKF_IMPL_FEATURE_ESTIMATOR_HPP
#define GVIO_MSCKF_IMPL_FEATURE_ESTIMATOR_HPP

namespace gvio {

template <typename T>
Eigen::Matrix<T, 3, 3>
CeresReprojectionError::quatToRot(const Eigen::Matrix<T, 4, 1> &q) const {
  const T q1 = q(0);
  const T q2 = q(1);
  const T q3 = q(2);
  const T q4 = q(3);

  Eigen::Matrix<T, 3, 3> R;
  R(0, 0) = 1.0 - 2.0 * pow(q2, 2.0) - 2.0 * pow(q3, 2.0);
  R(0, 1) = 2.0 * (q1 * q2 + q3 * q4);
  R(0, 2) = 2.0 * (q1 * q3 - q2 * q4);

  R(1, 0) = 2.0 * (q1 * q2 - q3 * q4);
  R(1, 1) = 1.0 - 2.0 * pow(q1, 2.0) - 2.0 * pow(q3, 2.0);
  R(1, 2) = 2.0 * (q2 * q3 + q1 * q4);

  R(2, 0) = 2.0 * (q1 * q3 + q2 * q4);
  R(2, 1) = 2.0 * (q2 * q3 - q1 * q4);
  R(2, 2) = 1.0 - 2.0 * pow(q1, 2.0) - 2.0 * pow(q2, 2.0);

  return R;
}

template <typename T>
Eigen::Matrix<T, 2, 1>
CeresReprojectionError::pixel2image(const T &pixel_x, const T &pixel_y) const {
  Eigen::Matrix<T, 2, 1> pt{(pixel_x - this->cx) / this->fx,
                            (pixel_y - this->cy) / this->fy};

  return pt;
}

template <typename T>
bool CeresReprojectionError::operator()(const T *const x, T *residual) const {
  // Inverse depth parameters
  const T alpha = x[0];
  const T beta = x[1];
  const T rho = x[2];

  // Form rotation matrix
  Eigen::Matrix<T, 3, 3> C_CiC0;
  int index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C_CiC0(i, j) = T(this->C_CiC0[index]);
      index++;
    }
  }

  // Form translation matrix
  Eigen::Matrix<T, 3, 1> t_Ci_CiC0;
  index = 0;
  for (int i = 0; i < 3; i++) {
    t_Ci_CiC0(i) = T(this->t_Ci_CiC0[index]);
    index++;
  }

  // Project estimated feature location to image plane
  const Eigen::Matrix<T, 3, 1> A(alpha, beta, T(1.0));
  const Eigen::Matrix<T, 3, 1> h = C_CiC0 * A + rho * t_Ci_CiC0;

  // Calculate reprojection error
  // -- Convert measurment to image coordinates
  const Eigen::Matrix<T, 2, 1> z =
      this->pixel2image(T(this->pixel_x), T(this->pixel_y));
  // -- Convert feature location to normalized coordinates
  const Eigen::Matrix<T, 2, 1> z_hat{h(0) / h(2), h(1) / h(2)};

  // Calculate residual error
  residual[0] = z(0) - z_hat(0);
  residual[1] = z(1) - z_hat(1);

  return true;
}

} // namespace gvio
#endif // GVIO_MSCKF_IMPL_FEATURE_ESTIMATOR_HPP
