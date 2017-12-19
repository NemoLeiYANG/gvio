/**
 * @file
 * @ingroup msckf
 */
#ifndef GVIO_MSCKF_FEATURE_ESTIMATOR_HPP
#define GVIO_MSCKF_FEATURE_ESTIMATOR_HPP

#include "gvio/util/util.hpp"

#include <ceres/ceres.h>

#include "gvio/quaternion/jpl.hpp"
#include "gvio/msckf/msckf.hpp"
#include "gvio/camera/camera_model.hpp"

namespace gvio {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * Feature estimator
 */
class FeatureEstimator {
public:
  const CameraModel *cam_model;
  const FeatureTrack track;
  const CameraStates track_cam_states;

  bool debug_mode = false;
  int max_iter = 30;

  FeatureEstimator(const CameraModel *cam_model,
                   const FeatureTrack &track,
                   const CameraStates &track_cam_states)
      : cam_model{cam_model}, track{track}, track_cam_states{track_cam_states} {
  }

  /**
   * Triangulate feature observed from camera C0 and C1 and return the
   * position relative to camera C0
   *
   * @param p1 Feature point 1 in pixel coordinates
   * @param p2 Feature point 2 in pixel coordinates
   * @param C_C0C1 Rotation matrix from frame C1 to C0
   * @param t_C0_C0C1 Translation vector from frame C0 to C1 expressed in C0
   * @param p_C0_f Initial feature position
   *
   * @returns 0 for success, -1 for failure
   */
  static int triangulate(const Vec2 &p1,
                         const Vec2 &p2,
                         const Mat3 &C_C0C1,
                         const Vec3 &t_C0_C0C1,
                         Vec3 &p_C0_f);

  /**
   * Calculate initial estimate
   *
   * @param p_C0_f Initial feature position
   * @returns 0 for success, -1 for failure
   */
  int initialEstimate(Vec3 &p_C0_f);

  /**
   * Jacobian
   *
   * @param x Optimization parameters
   * @returns Jacobian
   */
  MatX jacobian(const VecX &x);

  /**
   * Reprojection error
   *
   * @param x Optimization parameters
   * @returns Jacobian
   */
  VecX reprojectionError(const VecX &x);

  /**
   * Estimate feature position in global frame
   *
   * @param p_G_f Feature position in global frame
   * @returns 0 for success, -1 for failure
   */
  virtual int estimate(Vec3 &p_G_f);
};

/**
 * Vector to array
 *
 * @param v Vector
 * @returns Array
 */
inline double *vec2array(const VecX &v) {
  double *array = (double *) malloc(sizeof(double) * v.size());
  for (int i = 0; i < v.size(); i++) {
    array[i] = v(i);
    printf("%f\t", array[i]);
  }
  printf("\n");
  return array;
}

/**
 * Ceres-solver reprojection error
 */
struct CeresReprojectionError {
public:
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  double pixel_x = 0.0;
  double pixel_y = 0.0;

  CeresReprojectionError(const Mat3 &K, const cv::KeyPoint &keypoint)
      : fx(K(0, 0)), fy(K(1, 1)), cx(K(0, 2)), cy(K(1, 2)),
        pixel_x(keypoint.pt.x), pixel_y(keypoint.pt.y) {}

  /**
   * Form camera intrinsics matrix K
   *
   * @returns Camera intrinsics matrix K
   */
  template <typename T> Eigen::Matrix<T, 3, 3> formK() const {
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

  /**
   * Quaternion to rotation matrix R
   *
   * Page 9. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
   * Kalman filter for 3D attitude estimation." University of Minnesota,
   * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
   *
   * @param q JPL quaternion (x, y, z, w)
   * @returns Rotation matrix
   */
  template <typename T>
  Eigen::Matrix<T, 3, 3> quatToRot(const Eigen::Matrix<T, 4, 1> &q) const {
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

  /**
   * Calculate Bundle Adjustment Residual
   *
   * @param cam_q Camera rotation as quaternion (x, y, z, w)
   * @param cam_p Camera position (x, y, z)
   * @param landmark Landmark (x, y, z)
   * @param residual Calculated residual
   **/
  template <typename T>
  bool operator()(const T *const cam_q,
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
};

/**
 * Ceres-Solver based feature estimator
 */
class CeresFeatureEstimator : public FeatureEstimator {
public:
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  CeresFeatureEstimator(const CameraModel *cam_model,
                        const FeatureTrack track,
                        const CameraStates track_cam_states)
      : FeatureEstimator{cam_model, track, track_cam_states} {}

  /**
   * Estimate feature position in global frame
   *
   * @param p_G_f Feature position in global frame
   * @returns 0 for success, -1 for failure
   */
  int estimate(Vec3 &p_G_f);
};

/** @} group msckf */
} // namespace gvio
#endif // GVIO_MSCKF_FEATURE_ESTIMATOR_HPP
