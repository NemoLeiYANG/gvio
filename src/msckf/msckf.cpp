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
  std::cout << "N: " << this->N() << std::endl;
  std::cout << "x_imu_size: " << x_imu_size << std::endl;
  std::cout << "x_cam_size: " << x_cam_size << std::endl;
  std::cout << "P_imu_cam: " << this->P_imu_cam.rows() << "x"
            << this->P_imu_cam.cols() << std::endl;
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

  // this->imu_state.P = P.block(0, 0, x_imu_size, x_imu_size);
  this->P_cam = zeros(x_cam_size, x_cam_size);
  this->P_imu_cam = zeros(x_imu_size, x_cam_size);

  this->P_cam = P.block(x_imu_size, x_imu_size, x_cam_size, x_cam_size);
  this->P_imu_cam = P.block(0, x_imu_size, x_imu_size, x_cam_size);

  // std::cout << "x_imu_size: " << x_imu_size << std::endl;
  // std::cout << "x_cam_size: " << x_cam_size << std::endl;
  // std::cout << "P_imu_cam: ";
  // print_shape(this->P_imu_cam);
  // std::cout << "P_cam: ";
  // print_shape(this->P_cam);
  // std::cout << std::endl;

  // Add new camera state to sliding window by using current IMU pose
  // estimate to calculate camera pose
  // -- Create camera state in global frame
  const Vec4 imu_q_IG = this->imu_state.q_IG;
  const Vec3 imu_p_G = this->imu_state.p_G;
  const Vec4 cam_q_CG = quatlcomp(this->ext_q_CI) * imu_q_IG;
  const Vec3 cam_p_G = imu_p_G + C(imu_q_IG).transpose() * this->ext_p_IC;
  // -- Add camera state to sliding window
  this->cam_states.emplace_back(cam_p_G, cam_q_CG);
  this->counter_frame_id++;
}

CameraStates MSCKF::getTrackCameraStates(const FeatureTrack &track) {
  const FrameID fstart = track.frame_start;
  const FrameID fend = track.frame_end;
  const FrameID cstart = fstart - this->cam_states[0].frame_id;
  const FrameID cend = this->N() - (this->cam_states.back().frame_id - fend);

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

} // namespace gvio
