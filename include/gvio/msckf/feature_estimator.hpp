/**
 * @file
 * @ingroup msckf
 */
#ifndef GVIO_MSCKF_FEATURE_ESTIMATOR_HPP
#define GVIO_MSCKF_FEATURE_ESTIMATOR_HPP

#include <cmath>
#include <cfloat>

#include <ceres/ceres.h>

#include "gvio/util/util.hpp"
#include "gvio/quaternion/jpl.hpp"
#include "gvio/msckf/msckf.hpp"
#include "gvio/ceres/jpl_quaternion_parameterization.hpp"
#include "gvio/camera/camera_model.hpp"
#include "gvio/camera/pinhole_model.hpp"

namespace gvio {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * Linear Least Squares Triangulation
 *
 * Source:
 *     Hartley, R.I. and Sturm, P., "Triangulation", Computer vision and image
 *     understanding, 1997
 *
 * @param u1 homogenous image point (u,v,1)
 * @param P1 camera 1 matrix
 * @param u2 homogenous image point in 2nd camera
 * @param P2 camera 2 matrix
 *
 * @returns Estimated feature position
 */
Vec3 lls_triangulation(const Vec3 &u1,
                       const Mat34 &P1,
                       const Vec3 &u2,
                       const Mat34 &P2);

/**
 * Feature estimator
 */
class FeatureEstimator {
public:
  FeatureTrack track;
  CameraStates track_cam_states;

  bool debug_mode = false;
  int max_iter = 30;

  FeatureEstimator(const FeatureTrack &track,
                   const CameraStates &track_cam_states)
      : track{track}, track_cam_states{track_cam_states} {}

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
 * Auto-diff reprojection error
 */
struct AutoDiffReprojectionError {
public:
  // Camera extrinsics
  double C_CiC0[9];
  double t_Ci_CiC0[3];

  // Measurement
  double u = 0.0;
  double v = 0.0;

  AutoDiffReprojectionError(const Mat3 &C_CiC0,
                            const Vec3 &t_Ci_CiC0,
                            const Vec2 &kp);

  /**
   * JPL Quaternion to rotation matrix R
   *
   * Page 9. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
   * Kalman filter for 3D attitude estimation." University of Minnesota,
   * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
   *
   * @param q JPL quaternion (x, y, z, w)
   * @returns Rotation matrix
   */
  template <typename T>
  Eigen::Matrix<T, 3, 3> quatToRot(const Eigen::Matrix<T, 4, 1> &q) const;

  /**
   * Calculate Bundle Adjustment Residual
   *
   * @param x Inverse depth parameters (alpha, beta, rho)
   * @param residual Calculated residual
   **/
  template <typename T> bool operator()(const T *const x, T *residual) const;
};

/**
 * Analytical reprojection error
 */
struct AnalyticalReprojectionError : public ceres::SizedCostFunction<2, 3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Camera extrinsics
  Mat3 C_CiC0;
  Vec3 t_Ci_CiC0;

  // Measurement
  Vec2 keypoint;

  AnalyticalReprojectionError(const Mat3 &C_CiC0,
                              const Vec3 &t_Ci_CiC0,
                              const Vec2 &keypoint)
      : C_CiC0{C_CiC0}, t_Ci_CiC0{t_Ci_CiC0}, keypoint{keypoint} {}

  /**
   * Calculate reprojection residual
   *
   * @param parameters Optimization parameters
   * @param residuals Residuals
   * @param jacobians Jacobians
   **/
  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;
};

/**
 * Ceres-Solver based feature estimator
 */
class CeresFeatureEstimator : public FeatureEstimator {
public:
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  double x[3] = {0.0, 0.0, 0.0};

  CeresFeatureEstimator(const FeatureTrack &track,
                        const CameraStates &track_cam_states)
      : FeatureEstimator{track, track_cam_states} {}

  /**
   * Add residual block
   *
   * @param kp Keypoint
   * @param C_CiC0 Rotation matrix from frame C0 to Ci
   * @param t_Ci_CiC0 Translation vector from frame C0 to Ci expressed in Ci
   * @param x Optimization parameters
   */
  void addResidualBlock(const Vec2 &kp,
                        const Mat3 &C_CiC0,
                        const Vec3 &t_Ci_CiC0,
                        double *x);

  /**
   * Setup the optimization problem
   *
   * It performs the following 3 tasks:
   *
   * 1. Calculates an initial estimate of the landmark position
   * 2. Setup camera poses such that the first camera state is the origin,
   * since here we are performing a bundle adjustment of a feature track
   * relative to the first camera pose.
   * 3. Setup residual blocks
   *
   * @returns 0 for success, -1 for failure
   */
  int setupProblem();

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

#include "impl/feature_estimator.hpp"

#endif // GVIO_MSCKF_FEATURE_ESTIMATOR_HPP
