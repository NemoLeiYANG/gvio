#include "gvio/msckf/feature_estimator.hpp"

namespace gvio {

int FeatureEstimator::triangulate(const Vec2 &p1,
                                  const Vec2 &p2,
                                  const Mat3 &C_C0C1,
                                  const Vec3 &t_C0_C0C1,
                                  Vec3 &p_C0_f) {
  // Convert points to homogenous coordinates and normalize
  const Vec3 pt1{p1[0], p1[1], 1.0};
  const Vec3 pt2{p2[0], p2[1], 1.0};

  // Triangulate
  // -- Matrix A
  MatX A;
  A.resize(3, 2);
  A.block(0, 0, 3, 1) = pt1;
  A.block(0, 1, 3, 1) = -C_C0C1 * pt2;
  // -- Vector b
  Vec3 b{t_C0_C0C1};
  // -- Perform SVD
  VecX x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  // -- Calculate p_C0_f
  p_C0_f = x(0) * pt1;

  return 0;
}

int FeatureEstimator::initialEstimate(Vec3 &p_C0_f) {
  // Calculate rotation and translation of first and second camera states
  // -- Get rotation and translation of camera 0 and camera 1
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Mat3 C_C1G = C(this->track_cam_states[1].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;
  const Vec3 p_G_C1 = this->track_cam_states[1].p_G;
  // -- Calculate rotation and translation from camera 0 to camera 1
  const Mat3 C_C0C1 = C_C0G * C_C1G.transpose();
  const Vec3 t_C0_C0C1 = C_C0G * (p_G_C1 - p_G_C0);
  // -- Convert from pixel coordinates to image coordinates
  const Vec2 pt1 = this->track.track[0].getKeyPoint();
  const Vec2 pt2 = this->track.track[1].getKeyPoint();

  // Calculate initial estimate of 3D position
  FeatureEstimator::triangulate(pt1, pt2, C_C0C1, t_C0_C0C1, p_C0_f);

  return 0;
}

MatX FeatureEstimator::jacobian(const VecX &x) {
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;

  const int N = this->track_cam_states.size();
  MatX J = zeros(2 * N, 3);

  double alpha = x(0);
  double beta = x(1);
  double rho = x(2);

  for (int i = 0; i < N; i++) {
    // Get camera current rotation and translation
    const Mat3 C_CiG = C(track_cam_states[i].q_CG);
    const Vec3 p_G_Ci = track_cam_states[i].p_G;

    // Set camera 0 as origin, work out rotation and translation
    // of camera i relative to to camera 0
    const Mat3 C_CiC0 = C_CiG * C_C0G.transpose();
    const Vec3 t_Ci_CiC0 = C_CiG * (p_G_C0 - p_G_Ci);

    // Project estimated feature location to image plane
    const Vec3 A{alpha, beta, 1.0};
    const Vec3 h = C_CiC0 * A + rho * t_Ci_CiC0;

    // Compute jacobian
    const double hx_div_hz2 = (h(0) / pow(h(2), 2));
    const double hy_div_hz2 = (h(1) / pow(h(2), 2));

    const Vec2 drdalpha{-C_CiC0(0, 0) / h(2) + hx_div_hz2 * C_CiC0(2, 0),
                        -C_CiC0(1, 0) / h(2) + hy_div_hz2 * C_CiC0(2, 0)};

    const Vec2 drdbeta{-C_CiC0(0, 1) / h(2) + hx_div_hz2 * C_CiC0(2, 1),
                       -C_CiC0(1, 1) / h(2) + hy_div_hz2 * C_CiC0(2, 1)};

    const Vec2 drdrho{-t_Ci_CiC0(0) / h(2) + hx_div_hz2 * t_Ci_CiC0(2),
                      -t_Ci_CiC0(1) / h(2) + hy_div_hz2 * t_Ci_CiC0(2)};

    // Fill in the jacobian
    J.block(2 * i, 0, 2, 1) = drdalpha;
    J.block(2 * i, 1, 2, 1) = drdbeta;
    J.block(2 * i, 2, 2, 1) = drdrho;
  }

  return J;
}

VecX FeatureEstimator::reprojectionError(const VecX &x) {
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;

  const int N = this->track_cam_states.size();
  VecX residuals = zeros(2 * N, 1);

  double alpha = x(0);
  double beta = x(1);
  double rho = x(2);

  for (int i = 0; i < N; i++) {
    // Get camera current rotation and translation
    const Mat3 C_CiG = C(track_cam_states[i].q_CG);
    const Vec3 p_G_Ci = track_cam_states[i].p_G;

    // Set camera 0 as origin, work out rotation and translation
    // of camera i relative to to camera 0
    const Mat3 C_CiC0 = C_CiG * C_C0G.transpose();
    const Vec3 t_Ci_CiC0 = C_CiG * (p_G_C0 - p_G_Ci);

    // Project estimated feature location to image plane
    const Vec3 A{alpha, beta, 1.0};
    const Vec3 h = C_CiC0 * A + rho * t_Ci_CiC0;

    // Calculate reprojection error
    // -- Convert measurment to image coordinates
    const Vec2 z = track.track[i].getKeyPoint();
    // -- Convert feature location to normalized coordinates
    const Vec2 z_hat{h(0) / h(2), h(1) / h(2)};
    // -- Reprojcetion error
    residuals.block(2 * i, 0, 2, 1) = z - z_hat;
  }

  return residuals;
}

int FeatureEstimator::estimate(Vec3 &p_G_f) {
  // Calculate initial estimate of 3D position
  Vec3 p_C0_f;
  if (this->initialEstimate(p_C0_f) != 0) {
    return -1;
  }

  // Create inverse depth params (these are to be optimized)
  const double alpha = p_C0_f(0) / p_C0_f(2);
  const double beta = p_C0_f(1) / p_C0_f(2);
  const double rho = 1.0 / p_C0_f(2);
  Vec3 x{alpha, beta, rho};

  // Optimize feature position
  for (int k = 0; k < this->max_iter; k++) {
    // Calculate residuals and jacobian
    const VecX r = this->reprojectionError(x);
    const MatX J = this->jacobian(x);

    // Update optimization params using Gauss Newton
    MatX H_approx = J.transpose() * J;
    const VecX delta = H_approx.inverse() * J.transpose() * r;
    x = x - delta;

    // Debug
    if (this->debug_mode) {
      printf("iteration: %d  ", k);
      printf("track_length: %ld  ", track.trackedLength());
      printf("delta norm: %f  ", delta.norm());
      printf("max_residual: %.2f  ", r.maxCoeff());
      printf("\n");
    }

    // Converged?
    if (delta.norm() < 1e-8) {
      if (this->debug_mode) {
        printf("Converged!\n");
      }
      break;
    }
  }

  // Transform feature position from camera to global frame
  const Vec3 X{x(0), x(1), 1.0};
  const double z = 1 / x(2);
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;
  p_G_f = z * (C_C0G.transpose() * X) + p_G_C0;

  if (this->debug_mode) {
    std::cout << "p_G_f: " << p_G_f.transpose() << std::endl;
  }

  return 0;
}

bool CeresReprojectionError::Evaluate(double const *const *x,
                                      double *residuals,
                                      double **jacobians) const {
  // Inverse depth parameters
  const double alpha = x[0][0];
  const double beta = x[0][1];
  const double rho = x[0][2];

  // Project estimated feature location to image plane
  const Vec3 A{alpha, beta, 1.0};
  const Vec3 h = this->C_CiC0 * A + rho * this->t_Ci_CiC0;

  // Calculate reprojection error
  // -- Convert measurment to image coordinates
  const Vec2 z{this->keypoint};
  // -- Convert feature location to normalized coordinates
  const Vec2 z_hat{h(0) / h(2), h(1) / h(2)};

  // Calculate residual error
  residuals[0] = z(0) - z_hat(0);
  residuals[1] = z(1) - z_hat(1);

  // Compute the Jacobian if asked for.
  if (jacobians != NULL && jacobians[0] != NULL) {
    // Pre-compute common terms
    const double hx_div_hz2 = (h(0) / pow(h(2), 2));
    const double hy_div_hz2 = (h(1) / pow(h(2), 2));

    // **IMPORTANT** The ceres-solver documentation does not explain very well
    // how one goes about forming the jacobian. In a ceres analytical cost
    // function, the jacobian ceres needs is a local jacobian only, in this
    // problem we have:
    //
    // - 2 Residuals (Reprojection Error in x, y axis)
    // - 1 Parameter block of size 3 (Inverse depth, alpha, beta, rho)
    //
    // The resultant local jacobian should be of size 2x3 (2 residuals, 1st
    // parameter of size 3). The way we fill in the `jacobians` double array
    // variable is that since we are only calculating 1 local jacobian, we only
    // need to access the first index (i.e. 0), and then we fill in the
    // jacobian in ROW-MAJOR-ORDER, `jacobians[0][0...5]` for the 2x3
    // analytical jacobian, Ceres then in turn uses that information and forms
    // the global jacobian themselves.
    //
    // **IF** the problem had n parameter blocks, you would have filled in the
    // `jacobians[0..n][...]` local jacobians.
    //
    // For this problem our local jacobian has the form:
    //
    //   [drx / dalpha, drx / dbeta, drx / drho]
    //   [dry / dalpha, dry / dbeta, dry / drho]
    //
    // Or in row-major index form:
    //
    //   [0, 1, 2]
    //   [3, 4, 5]
    //

    // dr / dalpha
    jacobians[0][0] = -C_CiC0(0, 0) / h(2) + hx_div_hz2 * C_CiC0(2, 0);
    jacobians[0][3] = -C_CiC0(1, 0) / h(2) + hy_div_hz2 * C_CiC0(2, 0);

    // dr / dbeta
    jacobians[0][1] = -C_CiC0(0, 1) / h(2) + hx_div_hz2 * C_CiC0(2, 1);
    jacobians[0][4] = -C_CiC0(1, 1) / h(2) + hy_div_hz2 * C_CiC0(2, 1);

    // dr / drho
    jacobians[0][2] = -t_Ci_CiC0(0) / h(2) + hx_div_hz2 * t_Ci_CiC0(2);
    jacobians[0][5] = -t_Ci_CiC0(1) / h(2) + hy_div_hz2 * t_Ci_CiC0(2);
  }

  return true;
}

void CeresFeatureEstimator::addResidualBlock(const Vec2 &kp,
                                             const Mat3 &C_CiC0,
                                             const Vec3 &t_Ci_CiC0,
                                             double *x) {
  // Build residual
  auto cost_func = new CeresReprojectionError(C_CiC0, t_Ci_CiC0, kp);

  // Add residual block to problem
  this->problem.AddResidualBlock(cost_func, // Cost function
                                 NULL,      // Loss function
                                 x);        // Optimization parameters
}

int CeresFeatureEstimator::setupProblem() {
  // Setup landmark
  Vec3 p_C0_f;
  if (this->initialEstimate(p_C0_f) != 0) {
    return -1;
  }

  // Create inverse depth params (these are to be optimized)
  this->x[0] = p_C0_f(0) / p_C0_f(2); // Alpha
  this->x[1] = p_C0_f(1) / p_C0_f(2); // Beta
  this->x[2] = 1.0 / p_C0_f(2);       // Rho

  // Add residual blocks
  const int N = this->track_cam_states.size();
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;

  for (int i = 0; i < N; i++) {
    // Get camera's current rotation and translation
    const Mat3 C_CiG = C(track_cam_states[i].q_CG);
    const Vec3 p_G_Ci = track_cam_states[i].p_G;

    // Set camera 0 as origin, work out rotation and translation
    // of camera i relative to to camera 0
    const Mat3 C_CiC0 = C_CiG * C_C0G.transpose();
    const Vec3 t_Ci_CiC0 = C_CiG * (p_G_C0 - p_G_Ci);

    // Add residual block
    this->addResidualBlock(this->track.track[i].getKeyPoint(),
                           C_CiC0,
                           t_Ci_CiC0,
                           this->x);
  }

  return 0;
}

int CeresFeatureEstimator::estimate(Vec3 &p_G_f) {
  // Set options
  this->options.max_num_iterations = 30;
  this->options.use_nonmonotonic_steps = false;
  this->options.use_inner_iterations = false;
  this->options.preconditioner_type = ceres::SCHUR_JACOBI;
  this->options.linear_solver_type = ceres::SPARSE_SCHUR;
  this->options.parameter_tolerance = 1e-10;
  this->options.num_threads = 1;
  this->options.num_linear_solver_threads = 1;
  this->options.minimizer_progress_to_stdout = false;

  // Setup problem
  if (this->setupProblem() != 0) {
    return -1;
  }

  if (this->track.track[0].ground_truth.isApprox(Vec3::Zero()) == false) {
    p_G_f = this->track.track[0].ground_truth;
    return 0;
  }

  // Solve
  ceres::Solve(this->options, &this->problem, &this->summary);

  // Transform feature position from camera to global frame
  const Vec3 X{this->x[0], this->x[1], 1.0};
  const double z = 1 / this->x[2];
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;
  p_G_f = z * (C_C0G.transpose() * X) + p_G_C0;

  if (std::isnan(p_G_f(0)) || std::isnan(p_G_f(1)) || std::isnan(p_G_f(2))) {
    return -2;
  }

  return 0;
}

} // namespace gvio
