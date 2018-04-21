#include "gvio/msckf/feature_estimator.hpp"

namespace gvio {

Vec3 lls_triangulation(const Vec3 &u1,
                       const Mat34 &P1,
                       const Vec3 &u2,
                       const Mat34 &P2) {
  // Build matrix A for homogenous equation system Ax = 0, assume X = (x,y,z,1),
  // for Linear-LS method which turns it into a AX = B system, where:
  // - A is 4x3,
  // - X is 3x1
  // - B is 4x1

  // clang-format off
  MatX A = zeros(4, 3);
  A << u1(0) * P1(2, 0) - P1(0, 0), u1(0) * P1(2, 1) - P1(0, 1), u1(0) * P1(2, 2) - P1(0, 2),
       u1(1) * P1(2, 0) - P1(1, 0), u1(1) * P1(2, 1) - P1(1, 1), u1(1) * P1(2, 2) - P1(1, 2),
       u2(0) * P2(2, 0) - P2(0, 0), u2(0) * P2(2, 1) - P2(0, 1), u2(0) * P2(2, 2) - P2(0, 2),
       u2(1) * P2(2, 0) - P2(1, 0), u2(1) * P2(2, 1) - P2(1, 1), u2(1) * P2(2, 2) - P2(1, 2);

  Vec4 B{-(u1(0) * P1(2, 3) - P1(0,3)),
         -(u1(1) * P1(2, 3) - P1(1,3)),
         -(u2(0) * P2(2, 3) - P2(0,3)),
         -(u2(1) * P2(2, 3) - P2(1,3))};
  // clang-format on

  // SVD
  const auto svd_options = Eigen::ComputeThinU | Eigen::ComputeThinV;
  const Vec3 X = A.jacobiSvd(svd_options).solve(B);

  return X;
}

Vec3 lls_triangulation(const Vec2 &z1,
                       const Vec2 &z2,
                       const Mat3 &C_C0C1,
                       const Vec3 &t_C0_C0C1) {
  // Triangulate
  // -- Matrix A
  MatX A = zeros(3, 2);
  A.block(0, 0, 3, 1) = z1.homogeneous();
  A.block(0, 1, 3, 1) = -C_C0C1 * z2.homogeneous();

  // -- Vector b
  const Vec3 b{t_C0_C0C1};
  // -- Perform SVD
  const auto svd_options = Eigen::ComputeThinU | Eigen::ComputeThinV;
  const VecX x = A.jacobiSvd(svd_options).solve(b);
  // -- Calculate p_C0_f
  const Vec3 p_C0_f = x(0) * z1.homogeneous();

  return p_C0_f;
}

Vec3 lls_triangulation(const Vec2 &z1, const Vec2 &z2, const Mat4 T_C1_C0) {
  const Mat3 C_C1C0 = T_C1_C0.block(0, 0, 3, 3);
  const Vec3 t_C0_C1C0 = T_C1_C0.block(0, 3, 3, 1);
  const Vec3 m = C_C1C0 * z1.homogeneous();

  // Form A
  Vec2 A;
  A(0) = m(0) - z2(0) * m(2);
  A(1) = m(1) - z2(1) * m(2);

  // Form b
  Vec2 b;
  b(0) = z2(0) * t_C0_C1C0(2) - t_C0_C1C0(0);
  b(1) = z2(1) * t_C0_C1C0(2) - t_C0_C1C0(1);

  // Solve for depth
  const double depth = (A.transpose() * A).inverse() * A.transpose() * b;

  // Form initial feature position relative to camera 0
  Vec3 p_C0_f;
  p_C0_f(0) = z1(0) * depth;
  p_C0_f(1) = z1(1) * depth;
  p_C0_f(2) = depth;

  return p_C0_f;
}

FeatureEstimator::FeatureEstimator(const FeatureTrack &track,
                                   const CameraStates &track_cam_states)
    : track{track}, track_cam_states{track_cam_states} {}

FeatureEstimator::FeatureEstimator(const FeatureTrack &track,
                                   const CameraStates &track_cam_states,
                                   const Mat4 &T_C1_C0)
    : track{track}, track_cam_states{track_cam_states}, T_C1_C0{T_C1_C0} {}

int FeatureEstimator::triangulate(const Vec2 &z1,
                                  const Vec2 &z2,
                                  const Mat3 &C_C0C1,
                                  const Vec3 &t_C0_C0C1,
                                  Vec3 &p_C0_f) {
  // Convert points to homogenous coordinates and normalize
  const Vec3 pt1{z1[0], z1[1], 1.0};
  const Vec3 pt2{z2[0], z2[1], 1.0};

  // Form camera matrix P1
  const Mat34 P1 = I(3) * I(3, 4);

  // Form camera matrix P2
  Mat34 T2;
  T2.block(0, 0, 3, 3) = C_C0C1;
  T2.block(0, 3, 3, 1) = -C_C0C1 * t_C0_C0C1;
  const Mat34 P2 = I(3) * T2;

  // Perform linear least squares triangulation from 2 views
  p_C0_f = lls_triangulation(pt1, P1, pt2, P2);

  return 0;
}

int FeatureEstimator::initialEstimate(Vec3 &p_C0_f) {
  Mat3 C_C0C1;
  Vec3 t_C0_C0C1;
  Vec2 z1;
  Vec2 z2;

  if (this->track.type == MONO_TRACK) {
    // -- Calculate rotation and translation of first and second camera states
    const CameraState cam0 = this->track_cam_states.front();
    const CameraState cam1 = this->track_cam_states.back();
    // -- Get rotation and translation of camera 0 and camera 1
    const Mat3 C_C0G = C(cam0.q_CG);
    const Mat3 C_C1G = C(cam1.q_CG);
    const Vec3 p_G_C0 = cam0.p_G;
    const Vec3 p_G_C1 = cam1.p_G;
    // -- Calculate rotation and translation from camera 0 to camera 1
    C_C0C1 = C_C0G * C_C1G.transpose();
    t_C0_C0C1 = C_C0G * (p_G_C1 - p_G_C0);
    // -- Get observed image points
    z1 = this->track.track.front().getKeyPoint();
    z2 = this->track.track.back().getKeyPoint();

    // Triangulate
    if ((z1 - z2).norm() > 1.0e-3) {
      FeatureEstimator::triangulate(z1, z2, C_C0C1, t_C0_C0C1, p_C0_f);
    } else {
      LOG_WARN("Failed to triangulate feature!");
      return -1;
    }

  } else if (this->track.type == STEREO_TRACK) {
    // Triangulate feature point observed by stereo camera
    // -- Make sure the camera extrinsics are set
    assert(this->T_C1_C0.isApprox(zeros(4, 4)) == false);
    // -- Get observed image points
    z1 = this->track.track0.front().getKeyPoint();
    z2 = this->track.track1.front().getKeyPoint();

    p_C0_f = lls_triangulation(z1, z2, this->T_C1_C0);
    if (p_C0_f(2) < 1.0) {
      LOG_WARN("Bad initialization: [%.2f, %.2f, %.2f]",
               p_C0_f(0),
               p_C0_f(1),
               p_C0_f(2));
    }

  } else {
    FATAL("Invalid feature track type [%d]", track.type);
  }

  return 0;
}

int FeatureEstimator::checkEstimate(const Vec3 &p_G_f) {
  const int N = this->track_cam_states.size();

  // Pre-check
  if (std::isnan(p_G_f(0)) || std::isnan(p_G_f(1)) || std::isnan(p_G_f(2))) {
    return -1;
  }

  // Make sure feature is infront of camera all the way through
  for (int i = 0; i < N; i++) {
    // Transform feature from global frame to i-th camera frame
    const Mat3 C_CiG = C(this->track_cam_states[i].q_CG);
    const Vec3 p_Ci_f = C_CiG * (p_G_f - this->track_cam_states[i].p_G);

    // Check if feature is in-front of camera
    if (p_Ci_f(2) < 0.0 || p_Ci_f(2) > 100.0) {
      return -1;
    }
  }

  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_C0_f = C_C0G * (p_G_f - this->track_cam_states[0].p_G);
  std::cout << "p_C0_f: " << p_C0_f.transpose() << std::endl;

  return 0;
}

void FeatureEstimator::transformEstimate(const double alpha,
                                         const double beta,
                                         const double rho,
                                         Vec3 &p_G_f) {
  // Transform feature position from camera to global frame
  const Vec3 X{alpha, beta, 1.0};
  const double z = 1 / rho;
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;
  p_G_f = z * C_C0G.transpose() * X + p_G_C0;
}

MatX FeatureEstimator::jacobian(const VecX &x) {
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;

  double alpha = x(0);
  double beta = x(1);
  double rho = x(2);

  const int N = this->track_cam_states.size();
  MatX J = zeros(2 * N, 3);

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
  this->transformEstimate(x(0), x(1), x(2), p_G_f);
  if (std::isnan(p_G_f(0)) || std::isnan(p_G_f(1)) || std::isnan(p_G_f(2))) {
    return -2;
  }

  return 0;
}

AutoDiffReprojectionError::AutoDiffReprojectionError(const Mat3 &C_CiC0,
                                                     const Vec3 &t_Ci_CiC0,
                                                     const Vec2 &kp) {
  // Camera extrinsics
  mat2array(C_CiC0, this->C_CiC0);
  vec2array(t_Ci_CiC0, this->t_Ci_CiC0);

  // Measurement
  this->u = kp(0);
  this->v = kp(1);
}

bool AnalyticalReprojectionError::Evaluate(double const *const *x,
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
    const double hx_div_hz2 = (h(0) / (h(2), h(2)));
    const double hy_div_hz2 = (h(1) / (h(2), h(2)));

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
  if (this->method == "ANALYTICAL") {
    auto cost_func = new AnalyticalReprojectionError(C_CiC0, t_Ci_CiC0, kp);

    // Add residual block to problem
    this->problem.AddResidualBlock(cost_func, // Cost function
                                   NULL,      // Loss function
                                   x);        // Optimization parameters

  } else if (this->method == "AUTODIFF") {
    // Build residual
    auto residual = new AutoDiffReprojectionError(C_CiC0, t_Ci_CiC0, kp);

    // Build cost and loss function
    auto cost_func = new ceres::AutoDiffCostFunction<
        AutoDiffReprojectionError, // Residual
        2,                         // Size of residual
        3                          // Size of 1st parameter - inverse depth
        >(residual);

    // Add residual block to problem
    this->problem.AddResidualBlock(cost_func, // Cost function
                                   nullptr,   // Loss function
                                   x);        // Optimization parameters

  } else {
    LOG_ERROR("Invalid feature estimator method [%s]!", this->method.c_str());
    exit(-1);
  }
}

int CeresFeatureEstimator::setupProblem() {
  // Setup landmark
  Vec3 p_C0_f;
  if (this->initialEstimate(p_C0_f) != 0) {
    return -1;
  }

  // Cheat by initializing p_C0_f using ground truth data
  // if (this->track.track[0].ground_truth.isApprox(Vec3::Zero()) == false) {
  //   std::cout << "est: " << p_C0_f.transpose() << std::endl;
  //   const Vec3 p_G_f = this->track.track[0].ground_truth;
  //   const Vec3 p_G_C = this->track_cam_states[0].p_G;
  //   const Mat3 C_CG = C(this->track_cam_states[0].q_CG);
  //   p_C0_f = C_CG * (p_G_f - p_G_C);
  //   std::cout << "gnd: " << p_C0_f.transpose() << std::endl;
  //   std::cout << std::endl;
  // }

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
    if (this->track.type == MONO_TRACK) {
      this->addResidualBlock(this->track.track[i].getKeyPoint(),
                             C_CiC0,
                             t_Ci_CiC0,
                             this->x);
    } else if (this->track.type == STEREO_TRACK) {
      this->addResidualBlock(this->track.track0[i].getKeyPoint(),
                             C_CiC0,
                             t_Ci_CiC0,
                             this->x);

    } else {
      FATAL("Invalid feature track type [%d]", this->track.type);
    }
  }

  return 0;
}

int CeresFeatureEstimator::estimate(Vec3 &p_G_f) {
  // Set options
  this->options.max_num_iterations = 100;
  this->options.num_threads = 1;
  this->options.num_linear_solver_threads = 1;
  this->options.minimizer_progress_to_stdout = false;

  // Setup problem
  if (this->setupProblem() != 0) {
    return -1;
  }

  // Cheat by using ground truth data
  // if (this->track.track[0].ground_truth.isApprox(Vec3::Zero()) == false) {
  //   p_G_f = this->track.track[0].ground_truth;
  //   return 0;
  // }

  // Solve
  ceres::Solve(this->options, &this->problem, &this->summary);

  // Transform feature position from camera to global frame
  this->transformEstimate(this->x[0], this->x[1], this->x[2], p_G_f);

  // Check estimate
  if (this->checkEstimate(p_G_f) != 0) {
    LOG_WARN("Bad estimate p_G_f [%.2f, %.2f, %.2f]",
             p_G_f(0),
             p_G_f(1),
             p_G_f(2));
    return -2;
  }
  // Vec3 gnd = this->track.track[0].ground_truth;
  // std::cout << "gnd: " << gnd.transpose() << std::endl;
  // std::cout << "est: " << p_G_f.transpose() << std::endl;
  // std::cout << std::endl;

  return 0;
}

} // namespace gvio
