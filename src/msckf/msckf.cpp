#include "gvio/msckf/msckf.hpp"

#include <boost/math/distributions/chi_squared.hpp>

namespace gvio {

MSCKF::MSCKF() {
  // Create Chi-Square lookup table
  for (int i = 1; i < 100; ++i) {
    boost::math::chi_squared chi_squared_dist(i);
    this->chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
  }
}

int MSCKF::configure(const std::string &config_file) {
  // clang-format off
  // Load config file
  ConfigParser parser;
  IMUStateConfig imu_config;
  // -- General Settings
  parser.addParam("max_window_size", &this->max_window_size);
  parser.addParam("max_nb_tracks", &this->max_nb_tracks);
  parser.addParam("min_track_length", &this->min_track_length);
  parser.addParam("enable_ns_trick", &this->enable_ns_trick);
  parser.addParam("enable_qr_trick", &this->enable_ns_trick);
  // -- IMU Settings
  parser.addParam("imu.initial_covariance.q_init_var", &imu_config.q_init_var);
  parser.addParam("imu.initial_covariance.bg_init_var", &imu_config.bg_init_var);
  parser.addParam("imu.initial_covariance.v_init_var", &imu_config.v_init_var);
  parser.addParam("imu.initial_covariance.ba_init_var", &imu_config.ba_init_var);
  parser.addParam("imu.initial_covariance.p_init_var", &imu_config.p_init_var);
  parser.addParam("imu.process_noise.w_var", &imu_config.w_var);
  parser.addParam("imu.process_noise.dbg_var", &imu_config.dbg_var);
  parser.addParam("imu.process_noise.a_var", &imu_config.a_var);
  parser.addParam("imu.process_noise.dba_var", &imu_config.dba_var);
  parser.addParam("imu.constants.gravity_constant", &imu_config.g_G);
  // -- Camera Settings
  parser.addParam("camera.extrinsics.p_IC", &this->ext_p_IC);
  parser.addParam("camera.extrinsics.q_CI", &this->ext_q_CI);
  parser.addParam("camera.measurement_noise.img_var", &this->img_var);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }
  // clang-format on

  // Set IMU Settings
  this->imu_state = IMUState(imu_config);

  return 0;
}

VecX MSCKF::getState() {
  VecX state = zeros(9, 1);
  state.segment(0, 3) = this->imu_state.p_G;
  state.segment(3, 3) = this->imu_state.v_G;
  state.segment(6, 3) = quat2euler(this->imu_state.q_IG);
  return state;
}

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
  P = zeros(P_size, P_size);
  P.block(0, 0, x_imu_size, x_imu_size) = this->imu_state.P;
  P.block(0, x_imu_size, x_imu_size, x_cam_size) = this->P_imu_cam;
  P.block(x_imu_size, 0, x_cam_size, x_imu_size) = this->P_imu_cam.transpose();
  P.block(x_imu_size, x_imu_size, x_cam_size, x_cam_size) = this->P_cam;

  return P;
}

MatX MSCKF::J(const Vec4 &cam_q_CI,
              const Vec3 &cam_p_IC,
              const Vec4 &q_hat_IG,
              const int N) {
  const Mat3 C_CI = C(cam_q_CI);
  const Mat3 C_IG = C(q_hat_IG);

  MatX J = zeros(6, 15 + 6 * N);
  // -- First row --
  J.block(0, 0, 3, 3) = C_CI;
  // -- Second row --
  J.block(3, 0, 3, 3) = skew(C_IG.transpose() * cam_p_IC);
  J.block(3, 12, 3, 3) = I(3);

  return J;
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
    const Mat3 C_CiG = C(track_cam_states[i].q_CG);
    const Vec3 p_G_Ci = track_cam_states[i].p_G;
    const Vec3 p_C_f = C_CiG * (p_G_f - p_G_Ci);
    const double X = p_C_f(0);
    const double Y = p_C_f(1);
    const double Z = p_C_f(2);

    // dh / dg
    // clang-format off
    MatX dhdg = zeros(2, 3);
    dhdg << 1.0 / Z, 0.0, -X / (Z * Z),
            0.0, 1.0 / Z, -Y / (Z * Z);
    // clang-format on

    // Row start index
    const int rs = 2 * i;

    // Column start index
    const int cs_dhdq = x_imu_size + (x_cam_size * pose_idx);
    const int cs_dhdp = x_imu_size + (x_cam_size * pose_idx) + 3;

    // H_f_j measurement jacobian w.r.t feature
    H_f_j.block(rs, 0, 2, 3) = dhdg * C_CiG;

    // H_x_j measurement jacobian w.r.t state
    H_x_j.block(rs, cs_dhdq, 2, 3) = dhdg * skew(p_C_f);
    H_x_j.block(rs, cs_dhdp, 2, 3) = -dhdg * C_CiG;

    // TODO: Modify measurement jacobian according to OC-EKF

    // Update pose_idx
    pose_idx++;
  }
}

int MSCKF::initialize(const long ts) {
  this->last_updated = ts;
  this->augmentState();
  return 0;
}

int MSCKF::initialize(const long ts,
                      const Vec4 &q_IG,
                      const Vec3 &v_G,
                      const Vec3 &p_G) {
  this->last_updated = ts;
  this->imu_state.q_IG = q_IG;
  this->imu_state.v_G = v_G;
  this->imu_state.p_G = p_G;
  this->augmentState();

  return 0;
}

void MSCKF::augmentState() {
  // Camera pose jacobian
  const MatX J =
      this->J(this->ext_q_CI, this->ext_p_IC, this->imu_state.q_IG, this->N());

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
  const MatX P_fixed =
      (P + P.transpose()) / 2.0; // Fix covariance to be symmetric

  // Decompose covariance into its own constituents
  // clang-format off
  this->imu_state.P = P_fixed.block(0, 0, x_imu_size, x_imu_size);
  this->P_cam = P_fixed.block(x_imu_size, x_imu_size, P.rows() - x_imu_size, P.cols() - x_imu_size);
  this->P_imu_cam = P_fixed.block(0, x_imu_size, x_imu_size, P.cols() - x_imu_size);
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
  assert(this->N() != 0);

  // Calculate camera states where feature was observed
  const FrameID fstart = track.frame_start;
  const FrameID fend = track.frame_end;
  const FrameID cstart = fstart - this->cam_states[0].frame_id;
  const FrameID cend = this->N() - (this->cam_states.back().frame_id - fend);

  // Copy camera states
  auto first = this->cam_states.begin() + cstart;
  auto last = this->cam_states.begin() + cend;
  CameraStates track_cam_states{first, last};
  assert(track_cam_states.front().frame_id == track.frame_start);
  assert(track_cam_states.back().frame_id == track.frame_end);

  return track_cam_states;
}

int MSCKF::predictionUpdate(const Vec3 &a_m, const Vec3 &w_m, const long ts) {
  const double dt = (ts - this->last_updated) * 1e-9;
  this->imu_state.update(a_m, w_m, dt);
  this->P_cam = this->P_cam;
  this->P_imu_cam = this->imu_state.Phi * this->P_imu_cam;
  this->last_updated = ts;

  return 0;
}

int MSCKF::chiSquaredTest(const MatX &H, const VecX &r, const int dof) {
  const MatX P1 = H * this->P() * H.transpose();
  const MatX P2 = this->img_var * I(H.rows(), H.rows());
  const double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);

  if (gamma < this->chi_squared_table[dof]) {
    return 0;
  } else {
    return -1;
  }
}

int MSCKF::residualizeTrack(const FeatureTrack &track,
                            MatX &H_o_j,
                            VecX &r_o_j) {
  // Pre-check
  if (track.trackedLength() < (size_t) this->min_track_length) {
    return -1;
  } else if (track.trackedLength() >= (size_t) this->max_window_size) {
    return -1;
  }

  // Estimate j-th feature position in global frame
  const CameraStates track_cam_states = this->getTrackCameraStates(track);
  CeresFeatureEstimator feature_estimator(track, track_cam_states);
  Vec3 p_G_f;
  if (feature_estimator.estimate(p_G_f) != 0) {
    return -2;
  }

  // Calculate residuals
  VecX r_j = zeros(2 * track_cam_states.size(), 1);
  for (size_t i = 0; i < track_cam_states.size(); i++) {
    // Transform feature from global frame to i-th camera frame
    const Mat3 C_CG = C(track_cam_states[i].q_CG);
    const Vec3 p_C_f = C_CG * (p_G_f - track_cam_states[i].p_G);
    const double u = p_C_f(0) / p_C_f(2);
    const double v = p_C_f(1) / p_C_f(2);
    const Vec2 z_hat{u, v};

    // Transform idealized measurement
    const Vec2 z{track.track[i].getKeyPoint()};

    // Calculate reprojection error and add it to the residual vector
    const int rs = 2 * i;
    r_j.block(rs, 0, 2, 1) = z - z_hat;
  }

  // Form jacobian of measurement w.r.t both state and feature
  MatX H_f_j;
  MatX H_x_j;
  this->H(track, track_cam_states, p_G_f, H_f_j, H_x_j);

  // Perform Null Space Trick
  if (this->enable_ns_trick) {
    // Perform null space trick to decorrelate feature position error
    // away state errors by removing the measurement jacobian w.r.t.
    // feature position via null space projection [Section D:
    // Measurement Model, Mourikis2007]
    const unsigned int settings = Eigen::ComputeFullU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<MatX> svd(H_f_j, settings);
    const MatX A_j = svd.matrixU().rightCols(H_f_j.rows() - 3);
    H_o_j = A_j.transpose() * H_x_j;
    r_o_j = A_j.transpose() * r_j;

  } else {
    H_o_j = H_x_j;
    r_o_j = r_j;
  }

  // Peform chi squared test
  // const int dof = track.trackedLength() - 1;
  // if (this->chiSquaredTest(H_o_j, r_o_j, dof) != 0) {
  //   return -3;
  // }

  return 0;
}

int MSCKF::calcResiduals(const FeatureTracks &tracks, MatX &T_H, VecX &r_n) {
  // Residualize feature tracks
  MatX H_o;
  VecX r_o;

  for (auto track : tracks) {
    MatX H_j;
    VecX r_j;

    if (this->residualizeTrack(track, H_j, r_j) == 0) {
      // Stack measurement jacobian matrix and residual vector
      if (r_o.rows() > 0) {
        H_o = vstack(H_o, H_j);
        r_o = vstack(r_o, r_j);
      } else {
        H_o = H_j;
        r_o = r_j;
      }
    }
  }

  // No residuals, do not continue
  if (r_o.rows() == 0) {
    return -1;
  }
  if (r_o.maxCoeff() > 0.1) {
    LOG_WARN("Large residual! [%.4f]", r_o.maxCoeff());
  }

  // Reduce EKF measurement update computation with QR decomposition
  if (H_o.rows() > H_o.cols() && this->enable_qr_trick) {
    // Perform QR decompostion
    // -- Sparse QR method (faster)
    Eigen::SparseMatrix<double> H_sparse = H_o.sparseView();
    Eigen::SPQR<Eigen::SparseMatrix<double>> spqr_helper;
    spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL);
    spqr_helper.compute(H_sparse);

    MatX H_temp;
    VecX r_temp;
    (spqr_helper.matrixQ().transpose() * H_o).evalTo(H_temp);
    (spqr_helper.matrixQ().transpose() * r_o).evalTo(r_temp);

    T_H = H_temp.topRows(IMUState::size + this->N() * CameraState::size);
    r_n = r_temp.head(IMUState::size + this->N() * CameraState::size);

    // -- Dense QR method (slower)
    // Eigen::HouseholderQR<MatX> QR(H_o);
    // MatX Q = QR.householderQ();
    // MatX Q1 = Q.leftCols(IMUState::size + this->N() * CameraState::size);
    // T_H = Q1.transpose() * H_o;
    // r_n = Q1.transpose() * r_o;

  } else {
    T_H = H_o;
    r_n = r_o;
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
    const VecX dx_cam{dx.block(rs, 0, CameraState::size, 1)};
    this->cam_states[i].correct(dx_cam);
  }
}

void MSCKF::pruneCameraState() {
  // Pre-check
  if (this->N() <= this->max_window_size) {
    return;
  }

  // Prune camera states
  int prune_sz = this->N() - this->max_window_size;
  this->cam_states.erase(this->cam_states.begin(),
                         this->cam_states.begin() + prune_sz);

  // Adjust covariance matrix
  this->P_imu_cam = this->P_imu_cam.block(0,
                                          CameraState::size * prune_sz,
                                          IMUState::size,
                                          CameraState::size * this->N());
  this->P_cam =
      this->P_cam.block(CameraState::size * prune_sz,
                        CameraState::size * prune_sz,
                        this->P_cam.rows() - CameraState::size * prune_sz,
                        this->P_cam.cols() - CameraState::size * prune_sz);
}

FeatureTracks MSCKF::filterTracks(const FeatureTracks &tracks) {
  FeatureTracks filtered_tracks;
  for (auto track : tracks) {
    if (track.trackedLength() < (size_t) this->min_track_length) {
      continue;
    } else if (track.trackedLength() > (size_t) this->max_window_size) {
      continue;
    }

    filtered_tracks.push_back(track);
  }

  return filtered_tracks;
}

int MSCKF::measurementUpdate(const FeatureTracks &tracks) {
  // Add a camera state to state vector
  this->augmentState();

  // Filter feature tracks
  FeatureTracks filtered_tracks = this->filterTracks(tracks);
  if (filtered_tracks.size() == 0) {
    return 0;
  }

  // Calculate residuals
  MatX T_H;
  VecX r_n;
  if (this->calcResiduals(filtered_tracks, T_H, r_n) != 0) {
    LOG_WARN("No tracks made it through!");
    return 0;
  }

  // Calculate the Kalman gain.
  const MatX P = this->P();
  const MatX R_n = this->img_var * I(T_H.rows());
  const MatX S = T_H * P * T_H.transpose() + R_n;
  // -- Using Cholesky decomposition for matrix inversion
  // Note: Below is equiv to P * T_H^T * (T_H * P * T_H^T + R_n)^-1
  const MatX K = S.ldlt().solve(T_H * P).transpose();
  // -- Conventional Kalman gain calculation
  // const MatX K =
  //     P * T_H.transpose() * (T_H * P * T_H.transpose() + R_n).inverse();

  // Correct states
  const VecX dx = K * r_n;
  this->correctIMUState(dx);
  this->correctCameraStates(dx);
  LOG_INFO("Correct estimation!");

  // Update covariance matrices
  const MatX I_KH = I(K.rows(), T_H.cols()) - K * T_H;
  // -- Simplest, most unstable form
  // const MatX P_new = I_KH * P;
  // -- Symmetric and positive Joseph form
  // const MatX P_new = I_KH * P * I_KH.transpose() + K * R_n * K.transpose();
  // -- Upenn's version?
  const MatX P_new = ((I_KH * P) + (I_KH * P).transpose()) / 2.0;

  // Update covariance matrix
  this->imu_state.P = P_new.block(0, 0, IMUState::size, IMUState::size);
  this->P_cam = P_new.block(IMUState::size,
                            IMUState::size,
                            P_new.rows() - IMUState::size,
                            P_new.cols() - IMUState::size);
  this->P_imu_cam = P_new.block(0,
                                IMUState::size,
                                IMUState::size,
                                P_new.cols() - IMUState::size);

  // Prune camera state to maintain sliding window size
  this->pruneCameraState();

  return 0;
}

} // namespace gvio
