#ifndef GVIO_MSCKF_IMPL_FEATURE_ESTIMATOR_HPP
#define GVIO_MSCKF_IMPL_FEATURE_ESTIMATOR_HPP

namespace gvio {

template <typename T>
Eigen::Matrix<T, 3, 3> CeresReprojectionError::formK() const {
  Eigen::Matrix<T, 3, 3> K;
  K(0, 0) = T(this->fx);
  K(0, 1) = T(0.0);
  K(0, 2) = T(this->cx);

  K(1, 0) = T(0.0);
  K(1, 1) = T(this->fy);
  K(1, 2) = T(this->cy);

  K(2, 0) = T(0.0);
  K(2, 1) = T(0.0);
  K(2, 2) = T(1.0);
  return K;
}

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
bool CeresReprojectionError::operator()(const T *const cam_q,
                                        const T *const cam_p,
                                        const T *const landmark,
                                        T *residual) const {
  // Form camera intrinsics matrix K
  Eigen::Matrix<T, 3, 3> K = this->formK<T>();

  // Form rotation matrix from quaternion q = (x, y, z, w)
  const Eigen::Matrix<T, 4, 1> q{cam_q[0], cam_q[1], cam_q[2], cam_q[3]};
  const Eigen::Matrix<T, 3, 3> R = this->quatToRot<T>(q);

  // Form landmark
  const Eigen::Matrix<T, 3, 1> X{landmark[0], landmark[1], landmark[2]};

  // Form translation
  const Eigen::Matrix<T, 3, 1> t{cam_p[0], cam_p[1], cam_p[2]};

  // Project 3D point to image plane
  const Eigen::Matrix<T, 3, 1> est = K * R.transpose() * (X - t);

  // Convert projected point in homogenous coordinates to image coordinates
  Eigen::Matrix<T, 2, 1> est_pixel;
  est_pixel(0) = est(0) / est(2);
  est_pixel(1) = est(1) / est(2);

  // Calculate residual error
  residual[0] = ceres::abs(T(this->pixel_x) - est_pixel(0));
  residual[1] = ceres::abs(T(this->pixel_y) - est_pixel(1));

  return true;
}

} // namespace gvio
#endif // GVIO_MSCKF_IMPL_FEATURE_ESTIMATOR_HPP
