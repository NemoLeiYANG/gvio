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
  const Vec3 t_C1_C0C1 = C_C0G * (p_G_C1 - p_G_C0);
  // -- Convert from pixel coordinates to image coordinates
  const cv::Point2f kp1 = this->track.track[0].kp.pt;
  const cv::Point2f kp2 = this->track.track[1].kp.pt;
  const Vec2 pt1 = cam_model->pixel2image(kp1);
  const Vec2 pt2 = cam_model->pixel2image(kp2);

  // Calculate initial estimate of 3D position
  FeatureEstimator::triangulate(pt1, pt2, C_C0C1, t_C1_C0C1, p_C0_f);

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

    // clang-format off
    const Vec2 drdalpha{-C_CiC0(0, 0) / h(2) + (h(0) / pow(h(2), 2)) * C_CiC0(2, 0),
                        -C_CiC0(1, 0) / h(2) + (h(1) / pow(h(2), 2)) * C_CiC0(2, 0)};

    const Vec2 drdbeta{-C_CiC0(0, 1) / h(2) + (h(0) / pow(h(2), 2)) * C_CiC0(2, 1),
                       -C_CiC0(1, 1) / h(2) + (h(1) / pow(h(2), 2)) * C_CiC0(2, 1)};

    const Vec2 drdrho{-t_Ci_CiC0(0) / h(2) + (h(0) / pow(h(2), 2)) * t_Ci_CiC0(2),
                      -t_Ci_CiC0(1) / h(2) + (h(1) / pow(h(2), 2)) * t_Ci_CiC0(2)};
    // clang-format on

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
    const Vec2 z = this->cam_model->pixel2image(track.track[i].kp.pt);
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

  // Get ground truth
  // p_G_f = track.ground_truth;
  // Convert ground truth expressed in global frame
  // to be expressed in camera 0
  // const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  // const Vec3 p_G_C0 = this->track_cam_states[0].p_G;
  // p_C0_f = C_C0G * (p_G_f - p_G_C0);

  // Create inverse depth params (these are to be optimized)
  const double alpha = p_C0_f(0) / p_C0_f(2);
  const double beta = p_C0_f(1) / p_C0_f(2);
  const double rho = 1.0 / p_C0_f(2);
  Vec3 x0{alpha, beta, rho};

  //     // Optimize feature location
  //     args = (cam_model, track, track_cam_states)
  //     result = least_squares(self.reprojection_error,
  //                             theta_k,
  //                             args=args,
  //                             jac=self.jacobian,
  //                             verbose=1,
  //                             method="lm")
  //
  //     // if result.cost > 1e-4:
  //     //     return None

  // Transform feature position from camera to global frame
  // alpha, beta, rho = result.x.ravel() const double z = 1 / rho;
  const Vec3 X{alpha, beta, 1.0};
  const double z = 1 / rho;
  const Mat3 C_C0G = C(this->track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = this->track_cam_states[0].p_G;
  p_G_f = z * (C_C0G.transpose() * X) + p_G_C0;

  return 0;
}

} // namespace gvio
