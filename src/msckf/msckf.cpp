#include "gvio/msckf/msckf.hpp"

namespace gvio {

MatX MSCKF::P() {
  MatX P;

  // MSCKF not initialized yet
  if (this->N() == 0) {
    P = this->imu_state.P;
    return P;
  }

  // Calculate P matrix size
  const int x_imu_size = IMUState::size;
  const int x_cam_size = CameraState::size * this->N();
  const int P_size = x_imu_size + x_cam_size;

  // Form P
  P.resize(P_size, P_size);
  P.block(0, 0, x_imu_size, x_imu_size) = this->imu_state.P;
  P.block(0, x_imu_size, x_imu_size, x_cam_size) = this->P_imu_cam;
  P.block(x_imu_size, 0, x_cam_size, x_imu_size) = this->P_imu_cam.transpose();
  P.block(x_imu_size, x_imu_size, x_cam_size, x_cam_size) = this->P_cam;

  return P;
}

int MSCKF::N() { return (int) this->cam_states.size(); }

void MSCKF::H(const FeatureTrack &track,
              const CameraStates &track_cam_states,
              const Vec3 &p_G_f,
              MatX &H_f_j,
              MatX &H_x_j) {
  // Setup
  const double x_imu_size = IMUState::size;    // Size of imu state
  const double x_cam_size = CameraState::size; // Size of cam state

  const int N = this->N();             // Number of camera states
  const int M = track.trackedLength(); // Length of feature track

  // Measurement jacobian w.r.t feature
  H_f_j = zeros(2 * M, 3);

  // Measurement jacobian w.r.t state
  H_x_j = zeros(2 * M, x_imu_size + x_cam_size * N);

  // Pose index
  FrameID pose_idx = track.frame_start - this->cam_states[0].frame_id;

  // Form measurement jacobians
  for (int i = 0; i < M; i++) {
    // Feature position in camera frame
    const Mat3 C_CG = C(track_cam_states[i].q_CG);
    const Vec3 p_G_C = track_cam_states[i].p_G;
    const Vec3 p_C_f = C_CG * (p_G_f - p_G_C);
    const double X = p_C_f(0);
    const double Y = p_C_f(1);
    const double Z = p_C_f(2);

    // dh / dg
    MatX dhdg = zeros(2, 3);
    dhdg.block(0, 0, 1, 3) = Vec3{1.0 / Z, 0.0, -X / pow(Z, 2)};
    dhdg.block(1, 0, 1, 3) = Vec3{0.0, 1.0 / Z, -Y / pow(Z, 2)};

    // Row start index
    const int rs = 2 * i;

    // Column start index
    const int cs_dhdq = x_imu_size + (x_cam_size * pose_idx);
    const int cs_dhdp = x_imu_size + (x_cam_size * pose_idx) + 3;

    // H_f_j measurement jacobian w.r.t feature
    H_f_j.block(rs, 0, 2, 3) = dhdg * C_CG;

    // H_x_j measurement jacobian w.r.t state
    H_x_j.block(rs, cs_dhdq, 2, 3) = dhdg * skew(p_C_f);
    H_x_j.block(rs, cs_dhdp, 2, 3) = -dhdg * C_CG;

    // Update pose_idx
    pose_idx++;
  }
}

void MSCKF::augmentState() {
  // Camera pose jacobian
  const MatX J = this->imu_state.J(this->ext_q_CI,
                                   this->ext_p_IC,
                                   this->imu_state.q_IG,
                                   this->N());

  // Form temporary matrix to augment new camera state
  const int x_imu_size = IMUState::size;
  const int x_cam_size = CameraState::size * this->N();
  const int X0_size = x_imu_size + x_cam_size;
  const int X_rows = X0_size + J.rows();
  const int X_cols = X0_size;
  MatX X;
  X.resize(X_rows, X_cols);
  X.block(0, 0, X0_size, X0_size) = I(X0_size);
  X.block(X0_size, 0, J.rows(), J.cols()) = J;

  // Augment MSCKF covariance matrix (with new camera state)
  const MatX P = X * this->P() * X.transpose();

  // Decompose covariance into its own constituents
  // clang-format off
  this->imu_state.P = P.block(0, 0, x_imu_size, x_imu_size);
  this->P_cam = P.block(x_imu_size, x_imu_size, P.rows() - x_imu_size, P.cols() - x_imu_size);
  this->P_imu_cam = P.block(0, x_imu_size, x_imu_size, P.cols() - x_imu_size);
  // clang-format on

  // Add new camera state to sliding window by using current IMU pose
  // estimate to calculate camera pose
  // -- Create camera state in global frame
  const Vec4 imu_q_IG = this->imu_state.q_IG;
  const Vec3 imu_p_G = this->imu_state.p_G;
  const Vec4 cam_q_CG = quatlcomp(this->ext_q_CI) * imu_q_IG;
  const Vec3 cam_p_G = imu_p_G + C(imu_q_IG).transpose() * this->ext_p_IC;
  // -- Add camera state to sliding window
  this->cam_states.emplace_back(this->counter_frame_id, cam_p_G, cam_q_CG);
  this->counter_frame_id++;
}

CameraStates MSCKF::getTrackCameraStates(const FeatureTrack &track) {
  // Pre-check
  if (this->N() == 0) {
    LOG_ERROR("MSCKF not initialized yet! There are no camera states!");
    return CameraStates();
  }

  // Calculate camera states where feature was observed
  const FrameID fstart = track.frame_start;
  const FrameID fend = track.frame_end;
  const FrameID cstart = fstart - this->cam_states[0].frame_id;
  const FrameID cend = this->N() - (this->cam_states.back().frame_id - fend);

  // Copy camera states
  auto first = this->cam_states.begin() + cstart;
  auto last = this->cam_states.begin() + cend;
  CameraStates track_cam_states{first, last};

  return track_cam_states;
}

void MSCKF::predictionUpdate(const Vec3 &a_m,
                             const Vec3 &w_m,
                             const double dt) {
  this->imu_state.update(a_m, w_m, dt);
  this->P_cam = this->P_cam;
  this->P_imu_cam = this->imu_state.Phi * this->P_imu_cam;
}

int MSCKF::calTrackResiduals(const FeatureTrack &track,
                             MatX &H_j,
                             VecX &r_j,
                             MatX &R_j) {
  // Pre-check
  if (track.trackedLength() < this->min_track_length) {
    return -1;
  }

  // Estimate j-th feature position in global frame
  CameraStates track_cam_states = this->getTrackCameraStates(track);
  CeresFeatureEstimator feature_estimator(this->camera_model,
                                          track,
                                          track_cam_states);
  Vec3 p_G_f;
  if (feature_estimator.estimate(p_G_f) != 0) {
    return -2;
  }

  // Calculate residuals
  r_j = zeros(2 * this->N(), 1);

  for (int i = 0; i < this->N(); i++) {
    // Transform feature from global frame to i-th camera frame
    const Mat3 C_CG = C(track_cam_states[i].q_CG);
    const Vec3 p_C_f = C_CG * (p_G_f - track_cam_states[i].p_G);
    const double cu = p_C_f(0) / p_C_f(2);
    const double cv = p_C_f(1) / p_C_f(2);
    const Vec2 z_hat{cu, cv};

    // Transform idealized measurement
    const Vec2 z{this->camera_model->pixel2image(track.track[i].kp)};

    // Calculate reprojection error and add it to the residual vector
    const int rs = 2 * i;
    const int re = 2 * i + 2;
    r_j.block(rs, 0, re - rs, 1) = z - z_hat;
  }

  // Form jacobian of measurement w.r.t both state and feature
  MatX H_f_j;
  MatX H_x_j;
  this->H(track, track_cam_states, p_G_f, H_f_j, H_x_j);

  // Form the covariance matrix of different feature observations
  const int nb_residuals = r_j.rows();
  const Vec2 sigma_img{this->n_u, this->n_v};
  sigma_img.replicate(1, nb_residuals / 2.0);
  R_j = sigma_img.diagonal();

  // Perform Null Space Trick?
  if (this->enable_ns_trick) {
    // Perform null space trick to decorrelate feature position error
    // away state errors by removing the measurement jacobian w.r.t.
    // feature position via null space projection [Section D:
    // Measurement Model, Mourikis2007]
    const MatX A_j{nullspace(H_f_j.transpose())};
    H_j = A_j.transpose() * H_x_j;
    r_j = A_j.transpose() * r_j;
    R_j = A_j.transpose() * R_j * A_j;

  } else {
    H_j = H_x_j;
    r_j = r_j;
    R_j = R_j;
  }

  return 0;
}

int MSCKF::calResiduals(const FeatureTracks &tracks,
                        MatX &T_H,
                        VecX &r_n,
                        VecX &R_n) {
  // Residualize feature tracks
  MatX H;
  Vec3 r;
  MatX R;
  for (auto track : tracks) {
    MatX H_j;
    VecX r_j;
    MatX R_j;
    if (this->calTrackResiduals(track, H_j, r_j, R_j) == 0) {
      H = vstack(H, H_j);
      r = vstack(r, r_j);

      // R is a bit special, it is the covariance matrix so it has to be
      // stacked diagonally
      // clang-format off
      MatX R_copy(R.rows() + R_j.rows(), R.cols() + R_j.cols());
      R_copy.block(0, 0, R.rows(), R.cols()) = R;
      R_copy.block(0, R.cols(), R.rows(), R_j.cols()) = zeros(R.rows(), R_j.cols());
      R_copy.block(R.rows(), 0, R_j.rows(), R.cols()) = zeros(R_j.rows(), R.cols());
      R_copy.block(R.rows(), R.cols(), R_j.rows(), R_j.cols()) = R_j;
      // clang-format on
      R = R_copy;
    }
  }

  // No residuals, do not continue
  if (H.rows() == 0 && r.rows() == 0 and R.rows() == 0) {
    return -1;
  }

  // Perform QR decomposition?
  if (this->enable_qr_trick) {
    // Perform QR decomposition to reduce computation in actual EKF
    // update, since R for 10 features seen in 10 camera poses each
    // would yield a R of dimension 170 [Section E: EKF Updates,
    // Mourikis2007]
    const Eigen::HouseholderQR<MatX> qr(H);
    const MatX R = qr.matrixQR().triangularView<Eigen::Upper>();
    const MatX Q = qr.householderQ();
    // -- Find non-zero rows
    MatX T_H;
    MatX Q_1;
    for (int i = 0; i < R.rows(); i++) {
      if ((R.row(i).array() != 0.0).any()) {
        T_H = vstack(T_H, R.row(i));
        Q_1 = hstack(Q_1, Q.col(i));
      }
    }
    // -- Calculate residual
    r_n = Q_1.transpose() * r;
    R_n = Q_1.transpose() * R * Q_1;

  } else {
    T_H = H;
    r_n = r;
    R_n = R;
  }

  return 0;
}

void MSCKF::correctIMUState(const VecX &dx) {
  const VecX dx_imu = dx.block(0, 0, IMUState::size, 1);
  this->imu_state.correct(dx_imu);
}

void MSCKF::correctCameraStates(const VecX &dx) {
  for (int i = 0; i < this->N(); i++) {
    const int rs = IMUState::size + CameraState::size * i;
    const int re = IMUState::size + CameraState::size * i + CameraState::size;
    const VecX dx_cam{dx.block(rs, 0, re - rs, 1)};
    this->cam_states[i].correct(dx_cam);
  }
}

int MSCKF::measurementUpdate(FeatureTracks &tracks) {
  // Add a camera state to state vector
  this->augmentState();

  // Continue with EKF update?
  if (tracks.size() == 0) {
    return -1;
  }

  // Limit number of tracks
  if (tracks.size() > this->max_nb_tracks) {
    tracks.erase(tracks.begin(), tracks.begin() + 100);
  }

  // Calculate residuals
  MatX T_H;
  VecX r_n;
  VecX R_n;
  if (this->calResiduals(tracks, T_H, r_n, R_n) != 0) {
    return -2;
  }

  // Calculate Kalman gain
  const MatX P = this->P();
  const MatX K =
      P * T_H.transpose() * (T_H * P).inverse() * T_H.transpose() + R_n;

  // Correct states
  const VecX dx = K * r_n;
  this->correctIMUState(dx);
  this->correctCameraStates(dx);

  // Correct covariance matrices
  const MatX A = I(IMUState::size + CameraState::size * this->N()) - K * T_H;
  const MatX P_corrected = A * P * A.transpose() + K * R_n * K.transpose();
  this->imu_state.P = P_corrected.block(0, 0, IMUState::size, IMUState::size);
  this->P_cam = P_corrected.block(IMUState::size,
                                  IMUState::size,
                                  P_corrected.rows() - IMUState::size,
                                  P_corrected.cols() - IMUState::size);
  this->P_imu_cam = P_corrected.block(0,
                                      IMUState::size,
                                      IMUState::size,
                                      P_corrected.cols() - IMUState::size);

  return 0;
}

} // namespace gvio
