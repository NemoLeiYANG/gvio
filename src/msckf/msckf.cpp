#include "gvio/msckf/msckf.hpp"

namespace gvio {

void MSCKF::augmentState() {
  const int x_imu_size = this->imu_state.size;
  const int x_cam_size = this->N() ? this->cam_states[0].size : 0;

  // Camera pose Jacobian
  const MatX J = this->imu_state.J(this->ext_q_CI,
                                   this->ext_p_IC,
                                   this->imu_state.q_IG,
                                   this->N());

  // Augment MSCKF covariance matrix (with new camera state)
  X = np.block([ [I(x_imu_size + x_cam_size * this->N())], [J] ]) const MatX P =
      X * this->P() * X.transpose();
  // this->imu_state.P = P[0:x_imu_size, 0:x_imu_size]
  // this->P_cam = P[x_imu_size:, x_imu_size:]
  // this->P_imu_cam = P[0:x_imu_size, x_imu_size:]

  // // Add new camera state to sliding window by using current IMU pose
  // // estimate to calculate camera pose
  // // -- Create camera state in global frame
  // const Vec3 imu_q_IG = this->imu_state.q_IG;
  // const Vec4 imu_p_G = this->imu_state.p_G;
  // const Vec4 cam_q_CG = quatlcomp(this->ext_q_CI) * imu_q_IG;
  // const Vec3 cam_p_G = imu_p_G + C(imu_q_IG).transpose() * this->ext_p_IC;
  // // -- Add camera state to sliding window
  // this->cam_states.emplace_back{this->counter_frame_id, cam_q_CG, cam_p_G};
  // this->counter_frame_id += 1;
}

int MSCKF::getTrackCameraStates(const FeatureTrack &track,
                                std::vector<CameraState> &track_cam_states);
const FrameID frame_start = track.frame_start;
const FrameID frame_end = track.frame_end;
const FrameID index_start = frame_start - this->cam_states[0].frame_id;
const FrameID index_end =
    this->N() - (this->cam_states[-1].frame_id - frame_end);

// track_cam_states = this->cam_states[index_start:index_end]
// assert track_cam_states[0].frame_id == track.frame_start
// assert track_cam_states[-1].frame_id == track.frame_end

return 0;
}

MatX MSCKF::P() {
  MatX P;

  // if (this->N()) {
  //   P = np.block([[this->imu_state.P, this->P_imu_cam],
  //                 [this->P_imu_cam.T, this->P_cam]])
  // } else {
  //   P = this->imu_state.P;
  // }

  return P;
}

int MSCKF::N() { return (int) this->cam_states.size(); }

void MSCKF::H(MatX &H_f_j, MatX &H_x_j) {
  const double x_imu_size = this->imu_state.size;     // Size of imu state
  const double x_cam_size = this->cam_states[0].size; // Size of cam state

  const int N = this->N();              // Number of camera states
  const int M = track.tracked_length(); // Length of feature track

  // Measurement jacobian w.r.t feature
  MatX H_f_j = zeros(2 * M, 3);

  // Measurement jacobian w.r.t state
  MatX H_x_j = zeros(2 * M, x_imu_size + x_cam_size * N);

  // Pose index
  const FramID pose_idx = track.frame_start - this->cam_states[0].frame_id;
  assert track_cam_states[0].frame_id == track.frame_start;
  assert track_cam_states[-1].frame_id == track.frame_end;
  assert pose_idx == track.frame_start;
  assert(pose_idx + (M - 1)) == track.frame_end;

  // // Form measurement jacobians
  // for (int i = 0; i < M; i++) {
  //     // Feature position in camera frame
  //     C_CG = C(track_cam_states[i].q_CG);
  //     p_G_C = track_cam_states[i].p_G;
  //     p_C_f = dot(C_CG, (p_G_f - p_G_C));
  //     X, Y, Z = p_C_f.ravel();
  //
  //     // dh / dg
  //     dhdg = (1.0 / Z) * np.array([[1.0, 0.0, -X / Z],
  //                                   [0.0, 1.0, -Y / Z]])
  //
  //     // Row start and end index
  //     rs = 2 * i
  //     re = 2 * i + 2
  //
  //     // Column start and end index
  //     cs_dhdq = x_imu_size + (x_cam_size * pose_idx)
  //     ce_dhdq = x_imu_size + (x_cam_size * pose_idx) + 3
  //
  //     cs_dhdp = x_imu_size + (x_cam_size * pose_idx) + 3
  //     ce_dhdp = x_imu_size + (x_cam_size * pose_idx) + 6
  //
  //     // H_f_j measurement jacobian w.r.t feature
  //     H_f_j[rs:re, :] = dot(dhdg, C_CG)
  //
  //     // H_x_j measurement jacobian w.r.t state
  //     H_x_j[rs:re, cs_dhdq:ce_dhdq] = dot(dhdg, skew(p_C_f))
  //     H_x_j[rs:re, cs_dhdp:ce_dhdp] = dot(-dhdg, C_CG)
  //
  //     // Update pose_idx
  //     pose_idx += 1
  // }
}

void MSCKF::predictionUpdate(const Vec3 &a_m, const Ve3 &w_m, const double dt) {
  this->imu_state.update(a_m, w_m, dt);
  // this->P_imu, Phi =
  this->P_cam = this->P_cam;
  this->P_imu_cam = Phi * this->P_imu_cam;
}

} // namespace gvio
